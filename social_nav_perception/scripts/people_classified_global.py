#! /usr/bin/env python

import rospy
import tf2_ros
import math
from collections import deque
from geometry_msgs.msg import PoseStamped, Quaternion, Point
from zed_interfaces.msg import ObjectsStamped
from tf2_geometry_msgs import do_transform_pose
from social_nav_perception.msg import Person, PersonArray
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA

class PeopleInOdomFrame:
    def __init__(self):
        """
        Initializes the ROS node, publishers, subscriber, and necessary data structures.
        """
        rospy.init_node('people_in_odom_frame')

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # --- Publishers for Person and PersonArray ---
        self.moving_person_publisher = rospy.Publisher('/social_nav/moving_people/global', Person, queue_size=10)
        self.static_people_publisher = rospy.Publisher('/social_nav/static_people/global', PersonArray, queue_size=10)

        # --- Publishers for RViz Markers ---
        self.moving_person_marker_publisher = rospy.Publisher('/social_nav/moving_people/markers', Marker, queue_size=10)
        self.static_people_markers_publisher = rospy.Publisher('/social_nav/static_people/markers', MarkerArray, queue_size=10)

        self.people_subscriber = rospy.Subscriber('/zed2i/zed/obj_det/objects', ObjectsStamped, self.people_callback)

        self.person_history = {}
        self.FRAME_HISTORY_SIZE = 10

        rospy.loginfo("People tracking node started, publishing to /moving_person and /static_people.")
        rospy.loginfo("Publishing markers for RViz visualization on /moving_person_marker and /static_people_markers.")

    def euclidean_distance(self, pos1, pos2):
        """
        Calculates the Euclidean distance between two 3D points.
        """
        return math.sqrt((pos1.x - pos2.x)**2 + (pos1.y - pos2.y)**2 + (pos1.z - pos2.z)**2)

    def people_callback(self, msg):
        """
        Callback function that processes detected objects, tracks their movement,
        and publishes them to the appropriate topics.
        """
        all_detected_persons = []
        current_person_ids = set()

        try:
            transform = self.tf_buffer.lookup_transform('vision', 'zed2i_base_link', rospy.Time(0), rospy.Duration(1.0))

            for obj in msg.objects:
                if obj.label == 'Person':
                    person = Person()
                    person.id = obj.label_id

                    pose_stamped = PoseStamped()
                    pose_stamped.header = msg.header
                    pose_stamped.pose.position.x = obj.position[0]
                    pose_stamped.pose.position.y = obj.position[1]
                    pose_stamped.pose.position.z = obj.position[2]
                    pose_stamped.pose.orientation = Quaternion(0, 0, 0, 1)

                    transformed_pose = do_transform_pose(pose_stamped, transform)
                    person.pose = transformed_pose.pose
                    
                    all_detected_persons.append(person)
                    current_person_ids.add(person.id)

                    if person.id not in self.person_history:
                        self.person_history[person.id] = deque(maxlen=self.FRAME_HISTORY_SIZE)
                    
                    self.person_history[person.id].append(person.pose.position)

            for person_id in list(self.person_history.keys()):
                if person_id not in current_person_ids:
                    del self.person_history[person_id]

            person_movements = {}
            for person_id, history in self.person_history.items():
                if len(history) == self.FRAME_HISTORY_SIZE:
                    total_distance = 0
                    for i in range(1, len(history)):
                        total_distance += self.euclidean_distance(history[i-1], history[i])
                    person_movements[person_id] = total_distance
            
            moving_person_id = -1
            if person_movements:
                moving_person_id = max(person_movements, key=person_movements.get)

            static_people_array = PersonArray()
            static_people_array.header = msg.header
            
            static_markers = MarkerArray()

            for person in all_detected_persons:
                if person.id == moving_person_id:
                    self.moving_person_publisher.publish(person)
                    self.publish_moving_person_marker(person)
                else:
                    static_people_array.people.append(person)
                    static_markers.markers.append(self.create_static_person_marker(person))
            
            self.static_people_publisher.publish(static_people_array)
            self.static_people_markers_publisher.publish(static_markers)

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logerr(f"Transform error: {e}")

    def publish_moving_person_marker(self, person):
        marker = Marker()
        marker.header.frame_id = "vision"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "moving_person"
        marker.id = person.id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose = person.pose
        marker.scale.x = 0.5
        marker.scale.y = 0.5
        marker.scale.z = 0.5
        marker.color = ColorRGBA(1.0, 0.0, 0.0, 1.0)  # Red
        marker.lifetime = rospy.Duration(0.5)
        self.moving_person_marker_publisher.publish(marker)

    def create_static_person_marker(self, person):
        marker = Marker()
        marker.header.frame_id = "vision"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "static_people"
        marker.id = person.id
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose = person.pose
        marker.scale.x = 0.4
        marker.scale.y = 0.4
        marker.scale.z = 1.2
        marker.color = ColorRGBA(0.0, 1.0, 0.0, 1.0)  # Green
        marker.lifetime = rospy.Duration(0.5)
        return marker


if __name__ == '__main__':
    try:
        people_in_odom_frame = PeopleInOdomFrame()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass