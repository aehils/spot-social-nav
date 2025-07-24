#!/usr/bin/env python
# Simple script that subscribes to the /social_nav_perception/people_in_odom_frame
import rospy
import tf2_ros
from social_nav_perception.msg import PersonArray
from geometry_msgs.msg import PoseStamped, Quaternion
from tf2_geometry_msgs import do_transform_pose
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import math

class FollowHuman:
    def __init__(self):
        rospy.init_node('subscribe_to_data_camera', anonymous=True)

        rospy.loginfo("Initializing FollowHuman node...")
        self.sub = rospy.Subscriber('/social_nav/moving_people', PersonArray, self.person_callback)
        self.pub = rospy.Publisher('/spot/go_to_pose', PoseStamped, queue_size=10)
        self.rate = rospy.Rate(1.0)  # 1 Hz

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.person_to_follow = None
        self.desired_distance = 1.0  # distanza dietro la persona in metri
        self.target_frame = "vision"

    def person_callback(self, msg):
        if msg.people:
            self.person_to_follow = msg.people[0]
            pose = self.person_to_follow.pose.position
            rospy.loginfo("Detected person ID %d at (%.2f, %.2f, %.2f)",
                          self.person_to_follow.id, pose.x, pose.y, pose.z)
        else:
            self.person_to_follow = None

    def run(self):
        while not rospy.is_shutdown():
            if self.person_to_follow:
                try:
                    # crea PoseStamped da person_to_follow
                    person_ps = PoseStamped()
                    person_ps.header = self.person_to_follow.pose.header
                    person_ps.pose = self.person_to_follow.pose

                    # trasforma nel frame vision
                    tf_msg = self.tf_buffer.lookup_transform(
                        self.target_frame,
                        person_ps.header.frame_id,
                        rospy.Time(0), rospy.Duration(1.0))
                    vision_ps = do_transform_pose(person_ps, tf_msg)

                    # estrai x, y e yaw
                    x, y = vision_ps.pose.position.x, vision_ps.pose.position.y
                    q = vision_ps.pose.orientation
                    _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])

                    # calcola dietro
                    behind_x = x - self.desired_distance * math.cos(yaw)
                    behind_y = y - self.desired_distance * math.sin(yaw)

                    # crea goal
                    goal = PoseStamped()
                    goal.header.frame_id = self.target_frame
                    goal.header.stamp = rospy.Time.now()
                    goal.pose.position.x = behind_x
                    goal.pose.position.y = behind_y
                    goal.pose.position.z = 0.0
                    q_behind = quaternion_from_euler(0, 0, yaw)
                    goal.pose.orientation = Quaternion(*q_behind)

                    rospy.loginfo("Publishing behind-goal @ (%.2f, %.2f), yaw=%.1f°",
                                  behind_x, behind_y, math.degrees(yaw))
                    self.pub.publish(goal)

                except (tf2_ros.LookupException, tf2_ros.ExtrapolationException) as e:
                    rospy.logwarn("TF lookup failed: %s", e)
            else:
                rospy.logwarn("No person to follow detected.")
            self.rate.sleep()

if __name__ == '__main__':
    try:
        FollowHuman().run()
    except rospy.ROSInterruptException:
        pass