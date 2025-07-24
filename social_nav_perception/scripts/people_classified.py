#! /usr/bin/env python

import rospy
import tf2_ros
import math
import numpy as np
from collections import deque
from geometry_msgs.msg import PoseStamped, Quaternion
from zed_interfaces.msg import ObjectsStamped
from tf2_geometry_msgs import do_transform_pose
from social_nav_perception.msg import Person, PersonArray
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA

class PeopleInFrame:
    def __init__(self):
        """
        Initializes the ROS node, publishers, subscriber, and necessary data structures.
        """
        rospy.init_node('people_in_frame_node')

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # --- Publishers for different frames ---
        self.publishers = {
            'vision': {
                'moving': rospy.Publisher('/social_nav/moving_people/vision', Person, queue_size=10),
                'static': rospy.Publisher('/social_nav/static_people/vision', PersonArray, queue_size=10),
                'moving_marker': rospy.Publisher('/social_nav/moving_people/markers/vision', Marker, queue_size=10),
                'static_markers': rospy.Publisher('/social_nav/static_people/markers/vision', MarkerArray, queue_size=10)
            },
            'body': {
                'moving': rospy.Publisher('/social_nav/moving_people/body', Person, queue_size=10),
                'static': rospy.Publisher('/social_nav/static_people/body', PersonArray, queue_size=10),
                'moving_marker': rospy.Publisher('/social_nav/moving_people/markers/body', Marker, queue_size=10),
                'static_markers': rospy.Publisher('/social_nav/static_people/markers/body', MarkerArray, queue_size=10)
            }
        }

        self.people_subscriber = rospy.Subscriber('/zed2i/zed/obj_det/objects', ObjectsStamped, self.people_callback)

        self.person_history = {}
        self.FRAME_HISTORY_SIZE = 60
        self.target_frames = ['vision', 'body'] # Frames to transform to

        rospy.loginfo("People tracking node started.")
        rospy.loginfo("Publishing person data in 'vision' and 'body' frames.")

    def euclidean_distance(self, pos1, pos2):
        return math.sqrt((pos1.x - pos2.x)**2 + (pos1.y - pos2.y)**2 + (pos1.z - pos2.z)**2)

    def people_callback(self, msg):
        all_detected_persons_initial = []
        current_person_ids = set()

        moving_person_id = -1
        max_vel = 0
        for obj in msg.objects:
            if obj.label == 'Person':
                person = Person()
                person.id = obj.instance_id
                #person.velocity = np.linalg.norm(obj.velocity)
                p_vel = np.linalg.norm(obj.velocity)
                if p_vel > max_vel:
                    max_vel = p_vel
                    moving_person_id = person.id
                rospy.loginfo(f'Person {person.id} Velocity {p_vel}')

                pose_stamped = PoseStamped()
                pose_stamped.header = msg.header
                pose_stamped.pose.position.x = obj.position[0]
                pose_stamped.pose.position.y = obj.position[1]
                pose_stamped.pose.position.z = obj.position[2]
                # Assuming initial orientation is neutral
                pose_stamped.pose.orientation = Quaternion(0, 0, 0, 1)

                all_detected_persons_initial.append((person, pose_stamped))
                current_person_ids.add(person.id)

        # Update person history with positions from the message frame
        for person, pose_stamped in all_detected_persons_initial:
             if person.id not in self.person_history:
                self.person_history[person.id] = deque(maxlen=self.FRAME_HISTORY_SIZE)
             self.person_history[person.id].append(pose_stamped.pose.position)


        for person_id in list(self.person_history.keys()):
            if person_id not in current_person_ids:
                del self.person_history[person_id]

        person_movements = {}
        person_vels = {}
        for person_id, history in self.person_history.items():
            if len(history) == self.FRAME_HISTORY_SIZE:
                total_distance = self.euclidean_distance(history[-1], history[0])
                person_movements[person_id] = total_distance
                #rospy.loginfo(f'Person {person_id} Total distance= {total_distance}')

        # moving_person_id = max(person_movements, key=person_movements.get) if person_movements else -1
        # moving_person_id = max(person_vels, key=person_vels.get) if person_vels else -1

        for frame in self.target_frames:
            try:
                transform = self.tf_buffer.lookup_transform(frame, 'zed2i_base_link', rospy.Time(0), rospy.Duration(1.0))
                
                all_detected_persons_transformed = []
                for person, pose_stamped in all_detected_persons_initial:
                    transformed_pose_stamped = do_transform_pose(pose_stamped, transform)
                    
                    person_transformed = Person()
                    person_transformed.id = person.id
                    person_transformed.pose = transformed_pose_stamped.pose
                    all_detected_persons_transformed.append(person_transformed)

                static_people_array = PersonArray()
                static_people_array.header.frame_id = frame
                static_people_array.header.stamp = msg.header.stamp
                
                static_markers = MarkerArray()

                for p in all_detected_persons_transformed:
                    if p.id == moving_person_id:
                        self.publishers[frame]['moving'].publish(p)
                        self.publish_person_marker(p, frame, is_moving=True)
                    else:
                        static_people_array.people.append(p)
                        static_markers.markers.append(self.create_person_marker(p, frame, is_moving=False))
                
                self.publishers[frame]['static'].publish(static_people_array)
                if static_markers.markers:
                    self.publishers[frame]['static_markers'].publish(static_markers)

            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                rospy.logerr(f"Transform error to frame '{frame}': {e}")

    def publish_person_marker(self, person, frame_id, is_moving):
        marker = self.create_person_marker(person, frame_id, is_moving)
        if is_moving:
            self.publishers[frame_id]['moving_marker'].publish(marker)

    def create_person_marker(self, person, frame_id, is_moving):
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = rospy.Time.now()
        marker.ns = "people"
        marker.id = person.id
        marker.action = Marker.ADD
        marker.pose = person.pose
        marker.lifetime = rospy.Duration(0.05)

        if is_moving:
            marker.type = Marker.SPHERE
            marker.scale.x = 0.5
            marker.scale.y = 0.5
            marker.scale.z = 0.5
            marker.color = ColorRGBA(1.0, 0.0, 0.0, 1.0)  # Red
        else:
            marker.type = Marker.CUBE
            marker.scale.x = 0.4
            marker.scale.y = 0.4
            marker.scale.z = 1.2
            marker.color = ColorRGBA(0.0, 1.0, 0.0, 1.0)  # Green
        
        return marker


if __name__ == '__main__':
    try:
        people_in_frame = PeopleInFrame()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass