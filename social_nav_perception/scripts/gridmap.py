#!/usr/bin/env python3
# multicam_gridmap_people_height.py
# 多深度相机 → 高度分位数分割 → 环形缓存 → 占据栅格
# + 人员安全圈/椭圆写入 OGM & Marker 可视化（ROS1）

import rospy, tf2_ros, numpy as np, collections, time, math
import tf.transformations as tft
from sensor_msgs.msg import Image, CameraInfo
from nav_msgs.msg    import OccupancyGrid
from geometry_msgs.msg import Pose, Point
from cv_bridge import CvBridge
from threading import Lock
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA

# 你的自定义消息
from social_nav_perception.msg import Person, PersonArray


class MultiCamGrid:
    def __init__(self):
        rospy.init_node('multicam_gridmap')

        # ========== 参数 ==========
        gp = rospy.get_param
        cams = gp('~cameras', [])
        if not cams:
            rospy.logerr("~cameras 未配置")
            rospy.signal_shutdown("no cameras")
            return

        # 地图
        self.grid_frame   = gp('~grid_frame',     'vision')
        self.map_topic    = gp('~map_topic',      '/social_nav/map')
        self.res          = gp('~resolution',     0.05)
        self.size_x       = gp('~size_x',         200)
        self.size_y       = gp('~size_y',         200)
        self.publish_rate = gp('~publish_rate',   20.0)    # Hz

        # 深度采样
        self.stride     = gp('~stride',           1)
        self.min_depth  = gp('~min_depth',        0.30)
        self.max_depth  = gp('~max_depth',        4.0)
        self.voxel_size = gp('~voxel_size',       0.03)

        # 本体过滤盒
        self.robot_len  = gp('~robot_length',     0.35)
        self.robot_wid  = gp('~robot_width',      0.25)

        # 高度分位地面分割
        self.ground_qtl = gp('~ground_quantile',  0.05)
        self.ground_eps = gp('~ground_eps',       0.15)

        # 缓存
        self.cache_len   = gp('~cache_len',       12)
        self.decay_time  = gp('~decay_time',      0.6)     # s
        self.no_decay_d  = gp('~no_decay_distance', 4.0)

        # 人安全圈参数
        self.apply_people_to_grid  = gp('~people_to_grid', True)
        self.topic_moving          = gp('~moving_person_topic', '/social_nav/moving_people/global')
        self.topic_static          = gp('~static_people_topic', '/social_nav/static_people/global')

        self.safety_r_static       = gp('~safety_r_static', 0.6)
        self.safety_r_minor        = gp('~safety_r_minor',  0.4)
        self.major_base            = gp('~safety_major_base', 0.6)
        self.major_gain            = gp('~safety_major_gain', 1.2)
        self.min_speed_for_ellipse = gp('~min_speed_for_ellipse', 0.05)
        self.ellipse_points        = gp('~ellipse_points', 36)

        self.people_marker_ns      = gp('~people_marker_ns', 'people_safety')
        self.people_marker_topic   = gp('~people_marker_topic', '/social_nav/people_safety/markers')
        self.marker_alpha          = gp('~marker_alpha', 0.30)

        # ========== 内部状态 ==========
        self.map_size_xy = self.size_x * self.res
        self.half_x      = self.size_x * self.res / 2.0
        self.half_y      = self.size_y * self.res / 2.0

        self.bridge = CvBridge()
        self.tfbuf  = tf2_ros.Buffer(); tf2_ros.TransformListener(self.tfbuf)
        self.map_pub    = rospy.Publisher(self.map_topic, OccupancyGrid, queue_size=1, latch=True)
        self.marker_pub = rospy.Publisher(self.people_marker_topic, MarkerArray, queue_size=10)

        self.K_map   = {}   # frame_id -> K
        self.dir_map = {}   # frame_id -> (dir_x, dir_y)

        # 缓存/互斥
        self.cache = collections.deque(maxlen=self.cache_len)
        self.lock  = Lock()

        # 地图增量
        self.grid2d  = np.zeros((self.size_y, self.size_x), np.float32) # 0/1
        self.touched = []

        # 位姿/衰减
        self.pos        = np.zeros(3, dtype=np.float32)  # 当前相机/机器人位置
        self.pose_ref   = np.zeros(2, dtype=np.float32)
        self.decay_flag = 0
        self.init_t     = time.time()

        # 人信息
        self.people_lock = Lock()
        self.people      = {}  # id -> {'pos': [x,y,z], 'vel':[vx,vy], 't': time}
        self.people_hist = {}
        self.hist_len    = 8

        # ---------- 订阅 ----------
        for cam in cams:
            rospy.Subscriber(cam['info'],  CameraInfo, self.info_cb,  queue_size=1)
            rospy.Subscriber(cam['depth'], Image,      self.depth_cb, queue_size=1)
            rospy.loginfo(f"subscribe {cam['depth']}")

        rospy.Subscriber(self.topic_moving, Person,      self.moving_cb, queue_size=10)
        rospy.Subscriber(self.topic_static, PersonArray, self.static_cb, queue_size=10)

        # 定时器
        rospy.Timer(rospy.Duration(1.0/self.publish_rate), self.publish_grid)
        rospy.Timer(rospy.Duration(1.0), self.print_threshold)

        rospy.loginfo("multicam_gridmap_people_height ready → %s" % self.map_topic)

    # -------------------- 相机内参 --------------------
    def info_cb(self, msg):
        frame = msg.header.frame_id
        K = np.array(msg.K, dtype=np.float32).reshape(3,3)
        self.K_map[frame] = K
        if frame not in self.dir_map:
            width  = msg.width  // self.stride
            height = msg.height // self.stride
            u, v = np.meshgrid(np.arange(width)*self.stride,
                               np.arange(height)*self.stride)
            fx, fy, cx, cy = K[0,0], K[1,1], K[0,2], K[1,2]
            dir_x = (u - cx) / fx
            dir_y = (v - cy) / fy
            self.dir_map[frame] = (dir_x.astype(np.float32),
                                   dir_y.astype(np.float32))

    # -------------------- 深度回调 --------------------
    def depth_cb(self, img):
        frame = img.header.frame_id
        if frame not in self.K_map or frame not in self.dir_map:
            return
        dir_x, dir_y = self.dir_map[frame]

        depth = self.bridge.imgmsg_to_cv2(img)
        if depth.dtype != np.float32:
            depth = depth.astype(np.float32) / 1000.0
        depth = depth[::self.stride, ::self.stride]

        mask = np.isfinite(depth) & (depth > self.min_depth) & (depth < self.max_depth)
        if not mask.any():
            return

        z = depth[mask]
        x = dir_x[mask] * z
        y = dir_y[mask] * z
        pts_cam = np.stack((x, y, z), axis=1)

        # TF
        try:
            tr = self.tfbuf.lookup_transform(self.grid_frame, frame, rospy.Time(0), rospy.Duration(0.03))
        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException):
            return

        R = tft.quaternion_matrix([tr.transform.rotation.x,
                                   tr.transform.rotation.y,
                                   tr.transform.rotation.z,
                                   tr.transform.rotation.w])[:3,:3].astype(np.float32)
        T = np.array([tr.transform.translation.x,
                      tr.transform.translation.y,
                      tr.transform.translation.z], dtype=np.float32)

        pts = pts_cam @ R.T + T
        self.pos[:] = T

        # ROI & 车体过滤
        dx = pts[:,0] - T[0]; dy = pts[:,1] - T[1]
        hm = self.map_size_xy * 0.5
        roi = (np.abs(dx) < hm) & (np.abs(dy) < hm)
        if not roi.any(): return
        pts = pts[roi]; dx = dx[roi]; dy = dy[roi]

        body = (np.abs(dx) < self.robot_len*0.5) & (np.abs(dy) < self.robot_wid*0.5)
        pts = pts[~body]
        if pts.size == 0: return

        # 体素下采样
        if self.voxel_size > 0:
            pts = self.voxel_down(pts, self.voxel_size)
            if pts.size == 0: return

        # 高度分位地面分割
        z_rel = pts[:,2] - T[2]
        kth   = int(max(1, self.ground_qtl * z_rel.size))
        z0    = np.partition(z_rel, kth)[kth]
        thr   = z0 + self.ground_eps
        obs   = pts[z_rel >= thr]
        if obs.size == 0:
            return
        self._last_thr = thr

        # 缓存
        now = time.time()
        with self.lock:
            self.cache.append((now, obs))
            if self.decay_flag == 0:
                self.pose_ref[:] = self.pos[:2]
                self.decay_flag  = 1
            elif self.decay_flag == 1:
                d = math.hypot(self.pos[0]-self.pose_ref[0], self.pos[1]-self.pose_ref[1])
                if d >= self.no_decay_d:
                    self.decay_flag = 2

    # -------------------- 人员回调 --------------------
    def moving_cb(self, person: Person):
        self._update_person(person)

    def static_cb(self, arr: PersonArray):
        for p in arr.people:
            self._update_person(p)

    def _update_person(self, person: Person):
        p = person.pose.position
        t = rospy.get_time()

        with self.people_lock:
            hist = self.people_hist.get(person.id)
            if hist is None:
                hist = collections.deque(maxlen=self.hist_len)
                self.people_hist[person.id] = hist
            hist.append((t, p.x, p.y))

            vx, vy = 0.0, 0.0
            if len(hist) >= 2:
                (t0,x0,y0) = hist[0]
                (t1,x1,y1) = hist[-1]
                dt = max(1e-3, t1-t0)
                vx = (x1-x0)/dt
                vy = (y1-y0)/dt

            self.people[person.id] = {
                'pos': np.array([p.x, p.y, p.z], dtype=np.float32),
                'vel': np.array([vx, vy], dtype=np.float32),
                't':   t
            }
            # 清理长时间未更新的人员
            kill = [pid for pid, info in self.people.items() if t - info['t'] > 2.5]
            for pid in kill:
                self.people.pop(pid, None)
                self.people_hist.pop(pid, None)

    # -------------------- voxel down --------------------
    @staticmethod
    def voxel_down(xyz, sz):
        idx = np.floor(xyz / sz).astype(np.int32)
        key = (idx[:,0].astype(np.int64) << 42) + (idx[:,1].astype(np.int64) << 21) + idx[:,2]
        _, keep = np.unique(key, return_index=True)
        return xyz[keep]

    # -------------------- 发布地图 --------------------
    def publish_grid(self, _):
        # 合并缓存
        with self.lock:
            if not self.cache:
                return
            now = time.time()
            merged = []
            for t0, arr in self.cache:
                age  = now - t0
                dist = np.hypot(arr[:,0]-self.pos[0], arr[:,1]-self.pos[1])
                keep = (self.decay_flag < 2) | (age < self.decay_time) | (dist < self.no_decay_d)
                if keep.any():
                    merged.append(arr[keep])
        if not merged:
            return
        obs = np.vstack(merged)

        # 增量更新 depth 障碍
        if self.touched:
            self.grid2d.flat[self.touched] = 0.0

        cxs = ((obs[:,0] + self.half_x) / self.res).astype(np.int32)
        cys = ((obs[:,1] + self.half_y) / self.res).astype(np.int32)
        valid = (cxs>=0)&(cxs<self.size_x)&(cys>=0)&(cys<self.size_y)
        if valid.any():
            flat = cys[valid]*self.size_x + cxs[valid]
            flat = np.unique(flat)
            self.grid2d.flat[flat] = 1.0
            self.touched = flat.tolist()
        else:
            self.touched = []

        # --- 复制一份 int8 OGM 用于发布（不会改 grid2d） ---
        ogm = (self.grid2d > 0.5).astype(np.int8) * 100

        # --- 人安全圈写入 OGM ---
        markers = MarkerArray()
        if self.apply_people_to_grid or self.marker_pub.get_num_connections() > 0:
            # 仅在有人需要时才加锁
            with self.people_lock:
                tnow = rospy.Time.now()
                mid = 0
                for pid, info in self.people.items():
                    pos = info['pos']
                    vel = info['vel']
                    speed = np.linalg.norm(vel)

                    if speed < self.min_speed_for_ellipse:
                        # 圆形
                        r = self.safety_r_static
                        if self.apply_people_to_grid:
                            self.paint_circle(ogm, pos[0], pos[1], r)
                        m = self.make_cylinder(mid, pos, r, ColorRGBA(0.2,0.6,1.0,self.marker_alpha))
                        m.header.frame_id = self.grid_frame
                        m.header.stamp    = tnow
                        markers.markers.append(m)
                        mid += 1
                    else:
                        # 椭圆
                        a = self.major_base + self.major_gain * speed
                        b = self.safety_r_minor
                        yaw = math.atan2(vel[1], vel[0])
                        if self.apply_people_to_grid:
                            self.paint_ellipse(ogm, pos[0], pos[1], a, b, yaw)
                        m = self.make_ellipse_marker(mid, pos, a, b, yaw,
                                                     ColorRGBA(1.0,0.4,0.2,self.marker_alpha))
                        m.header.frame_id = self.grid_frame
                        m.header.stamp    = tnow
                        markers.markers.append(m)
                        mid += 1

        # 发布 OGM
        grid = OccupancyGrid()
        grid.header.stamp    = rospy.Time.now()
        grid.header.frame_id = self.grid_frame
        grid.info.resolution = self.res
        grid.info.width      = self.size_x
        grid.info.height     = self.size_y
        grid.info.origin     = Pose()
        grid.info.origin.position.x = -self.half_x
        grid.info.origin.position.y = -self.half_y
        grid.info.origin.orientation.w = 1.0
        grid.data = ogm.ravel().tolist()
        self.map_pub.publish(grid)

        # 发布 Marker
        if markers.markers:
            self.marker_pub.publish(markers)

    # -------------------- 栅格绘制（circle/ellipse） --------------------
    def world_to_cell(self, x, y):
        cx = int((x + self.half_x) / self.res)
        cy = int((y + self.half_y) / self.res)
        return cx, cy

    def paint_circle(self, ogm_int8, x, y, r):
        rr = r / self.res
        cx, cy = self.world_to_cell(x, y)
        x0 = max(0, int(cx - rr - 1))
        x1 = min(self.size_x-1, int(cx + rr + 1))
        y0 = max(0, int(cy - rr - 1))
        y1 = min(self.size_y-1, int(cy + rr + 1))
        rr2 = rr * rr
        for iy in range(y0, y1+1):
            dy = iy - cy
            row = ogm_int8[iy]
            for ix in range(x0, x1+1):
                dx = ix - cx
                if dx*dx + dy*dy <= rr2:
                    row[ix] = 100

    def paint_ellipse(self, ogm_int8, x, y, a, b, yaw):
        a_c = a / self.res
        b_c = b / self.res
        cx, cy = self.world_to_cell(x, y)
        cos_y = math.cos(yaw)
        sin_y = math.sin(yaw)

        rx = int(a_c) + 2
        ry = int(b_c) + 2
        x0 = max(0, cx - rx)
        x1 = min(self.size_x-1, cx + rx)
        y0 = max(0, cy - ry)
        y1 = min(self.size_y-1, cy + ry)

        for iy in range(y0, y1+1):
            dy = iy - cy
            row = ogm_int8[iy]
            for ix in range(x0, x1+1):
                dx = ix - cx
                xr =  dx * cos_y + dy * sin_y
                yr = -dx * sin_y + dy * cos_y
                if (xr*xr)/(a_c*a_c) + (yr*yr)/(b_c*b_c) <= 1.0:
                    row[ix] = 100

    # -------------------- Marker 构造 --------------------
    def make_cylinder(self, mid, pos, r, color):
        m = Marker()
        m.ns = self.people_marker_ns
        m.id = mid
        m.type = Marker.CYLINDER
        m.action = Marker.ADD
        m.pose.position.x = pos[0]
        m.pose.position.y = pos[1]
        m.pose.position.z = 0.0
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = 2*r
        m.scale.z = 0.05
        m.color = color
        m.lifetime = rospy.Duration(0.3)
        return m

    def make_ellipse_marker(self, mid, pos, a, b, yaw, color):
        m = Marker()
        m.ns = self.people_marker_ns
        m.id = mid
        m.type = Marker.LINE_STRIP
        m.action = Marker.ADD
        m.pose.orientation.w = 1.0
        m.scale.x = 0.03
        cos_y = math.cos(yaw)
        sin_y = math.sin(yaw)
        pts = []
        for i in range(self.ellipse_points+1):
            th = 2*math.pi*i/self.ellipse_points
            ex = a * math.cos(th)
            ey = b * math.sin(th)
            rx = ex*cos_y - ey*sin_y + pos[0]
            ry = ex*sin_y + ey*cos_y + pos[1]
            pts.append(Point(x=rx, y=ry, z=0.0))
        m.points = pts
        m.color = color
        m.lifetime = rospy.Duration(0.3)
        return m

    # -------------------- 日志 --------------------
    def print_threshold(self, _):
        if hasattr(self, "_last_thr"):
            rospy.loginfo_throttle(1.0,
                f"[ground] qtl={self.ground_qtl} thr(rel)={self._last_thr:.3f} | people={len(self.people)} | cache={len(self.cache)}")

# -------------------- main --------------------
if __name__ == '__main__':
    try:
        MultiCamGrid()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
