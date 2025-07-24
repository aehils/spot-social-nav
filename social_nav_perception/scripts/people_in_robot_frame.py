#! /usr/bin/env python

import rospy
import tf2_ros
from geometry_msgs.msg import PoseStamped, Quaternion
from zed_interfaces.msg import ObjectsStamped
from tf2_geometry_msgs import do_transform_pose
from social_nav_perception.msg import Person, PersonArray 

class PeopleInOdomFrame:
    def __init__(self):
        rospy.init_node('people_in_odom_frame')

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.people_subscriber = rospy.Subscriber('/zed2i/zed/obj_det/objects', ObjectsStamped, self.people_callback)
        self.odom_publisher = rospy.Publisher('/social_nav/people_in_robot_frame', PersonArray, queue_size=10)

    def people_callback(self, msg):
        person_array = PersonArray()
        person_array.header = msg.header

        try:
            transform = self.tf_buffer.lookup_transform('vision', 'zed2i_base_link', rospy.Time(0), rospy.Duration(1.0))
            for obj in msg.objects:
                if obj.label == 'Person':
                    person = Person()
                    person.id = obj.instance_id
                    pose_stamped = PoseStamped()
                    pose_stamped.header = msg.header
                    pose_stamped.pose.position.x = obj.position[0]
                    pose_stamped.pose.position.y = obj.position[1]
                    pose_stamped.pose.position.z = obj.position[2]
                    pose_stamped.pose.orientation = Quaternion(0, 0, 0, 1)
                    transformed_pose = do_transform_pose(pose_stamped, transform)
                    person.pose = transformed_pose.pose
                    person_array.people.append(person)

            self.odom_publisher.publish(person_array)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logerr(f"Transform error: {e}")

if __name__ == '__main__':
    try:
        people_in_odom_frame = PeopleInOdomFrame()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
