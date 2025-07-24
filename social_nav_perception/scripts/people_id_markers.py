#! /usr/bin/env python

import rospy
import tf2_ros
from visualization_msgs.msg import Marker, MarkerArray
from zed_interfaces.msg import ObjectsStamped
from geometry_msgs.msg import PoseStamped, Point
from tf2_geometry_msgs import do_transform_pose

class PersonLabelPublisher:
    """
    This class subscribes to ZED ObjectsStamped messages and publishes
    visualization_msgs/MarkerArray to display person IDs as text in RViz.
    """
    def __init__(self):
        rospy.init_node('person_label_publisher')

        # === Parameters ===
        # The fixed frame in which to publish the markers (e.g., 'odom', 'map')
        self.target_frame = rospy.get_param('~target_frame', 'odom')
        # The font size for the label in RViz
        self.text_size = rospy.get_param('~text_size', 0.15) 

        # === TF2 Setup ===
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # === Publishers and Subscribers ===
        self.objects_subscriber = rospy.Subscriber(
            '/zed2i/zed/obj_det/objects', 
            ObjectsStamped, 
            self.objects_callback
        )
        self.label_publisher = rospy.Publisher(
            '/social_nav/person_labels', 
            MarkerArray, 
            queue_size=10
        )
        
        rospy.loginfo("Person label publisher started.")
        rospy.loginfo(f"Publishing markers in '{self.target_frame}' frame.")

    def objects_callback(self, msg):
        """
        Processes detected objects and publishes markers for each person.
        """
        marker_array = MarkerArray()

        try:
            # Get the transform from the source frame (where objects are detected)
            # to the target frame (for visualization in RViz).
            transform = self.tf_buffer.lookup_transform(
                self.target_frame, 
                msg.header.frame_id, 
                rospy.Time(0), 
                rospy.Duration(1.0)
            )

            for obj in msg.objects:
                if obj.label == 'Person':
                    # --- 1. Calculate the center of the 3D bounding box ---
                    # The bounding box has 8 corners. The center is their average.
                    center_x = sum(corner.kp[0] for corner in obj.bounding_box_3d.corners) / 8.0
                    center_y = sum(corner.kp[1] for corner in obj.bounding_box_3d.corners) / 8.0
                    center_z = sum(corner.kp[2] for corner in obj.bounding_box_3d.corners) / 8.0

                    # --- 2. Create a PoseStamped for the center point for transformation ---
                    pose_stamped = PoseStamped()
                    pose_stamped.header = msg.header
                    pose_stamped.pose.position = Point(center_x, center_y, center_z)
                    pose_stamped.pose.orientation.w = 1.0 # Neutral orientation

                    # --- 3. Transform the pose to the target frame ---
                    transformed_pose_stamped = do_transform_pose(pose_stamped, transform)

                    # --- 4. Create the Marker ---
                    marker = Marker()
                    marker.header.frame_id = self.target_frame
                    marker.header.stamp = msg.header.stamp
                    marker.ns = "person_labels"
                    marker.id = obj.instance_id # Use the unique tracking ID for the marker ID
                    marker.type = Marker.TEXT_VIEW_FACING
                    marker.action = Marker.ADD
                    
                    # Set the marker's pose to the transformed center of the bounding box
                    marker.pose = transformed_pose_stamped.pose
                    
                    # Set the label text
                    marker.text = f"Person {obj.instance_id}"

                    # Set marker properties (scale, color, lifetime)
                    marker.scale.z = self.text_size # Text height
                    marker.color.a = 1.0  # Must be non-zero for the marker to be visible
                    marker.color.r = 1.0
                    marker.color.g = 1.0
                    marker.color.b = 1.0
                    
                    # The marker will be automatically deleted after this duration
                    marker.lifetime = rospy.Duration(0.5)

                    marker_array.markers.append(marker)

            if marker_array.markers:
                self.label_publisher.publish(marker_array)

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn_throttle(2.0, f"Failed to transform person labels: {e}")

if __name__ == '__main__':
    try:
        PersonLabelPublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass