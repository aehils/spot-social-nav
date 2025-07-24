#!/usr/bin/env python
# Simple script that subscribes to the /social_nav_perception/people_in_odom_frame
import rospy
from social_nav_perception.msg import PersonArray, Person
from geometry_msgs.msg import PoseStamped, Quaternion
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from math import atan2, sin, cos, sqrt 

class FollowHuman:
    def __init__(self):
        # Initialize ROS node only if it hasn't been initialized yet
        rospy.init_node('subscribe_to_data_camera', anonymous=True)
        self.readingData_Person = rospy.Subscriber('/social_nav/moving_people/body', Person, self.person_callback)
        #self.readingData_People = rospy.Subscriber('/social_nav/static_people', PersonArray, self.people_callback)
        self.pub = rospy.Publisher('/spot/go_to_pose', PoseStamped, queue_size=10)
        self.rate = rospy.Rate(10) # 1 Hz = 1 second interval
        self.static_people = None
        self.person_to_follow = None
        self.R = 0.9

    def person_callback(self, msg):
        if msg:
            self.person_to_follow = msg # Assuming we want to follow the first detected person and that the "0" is the ID of the person
            # rospy.loginfo("Detected person to follow with ID: %d at position (%.2f, %.2f, %.2f)", 
            #              self.person_to_follow.id, 
            #              self.person_to_follow.pose.position.x,
            #              self.person_to_follow.pose.position.y,
            #              self.person_to_follow.pose.position.z)
        else:
            self.person_to_follow = None

    # def people_callback(self, msg):
    #     if msg.people:
    #         self.static_people = msg.people
    #         rospy.loginfo("Detected static people:")
    #         for person in self.static_people:
    #             rospy.loginfo(" - Person ID: %d at position (%.2f, %.2f, %.2f)", 
    #                           person.id, 
    #                           person.pose.position.x,
    #                           person.pose.position.y,
    #                           person.pose.position.z)
    #     else:
    #         self.static_people = None

    def find_angle(self, x, y):
        theta_tf = atan2(y, x)
        rospy.loginfo("Angle is " + str(theta_tf))
        theta = quaternion_from_euler(0, 0, theta_tf)
        return theta
    
    def set_tracking_dist(self, x, y):
        R = self.R
        theta = atan2(y,x)
        dist_person2spot = sqrt(x**2 + y**2)
        dist_r2spot = dist_person2spot - R
        x_tracking = dist_r2spot * cos(theta)
        y_tracking = dist_r2spot * sin(theta)
        return x_tracking, y_tracking
    
    def run(self):
        while not rospy.is_shutdown():
            if self.person_to_follow:
                goal = PoseStamped()
                goal.header.frame_id = "body"  # The frame of reference is 'vision'
                x_goal_pos , y_goal_pos = self.set_tracking_dist(self.person_to_follow.pose.position.x, self.person_to_follow.pose.position.y)
                goal.pose.position.x = x_goal_pos
                goal.pose.position.y = y_goal_pos
                q = self.find_angle(self.person_to_follow.pose.position.x, self.person_to_follow.pose.position.y)
                goal.pose.orientation.x = q[0]
                goal.pose.orientation.y = q[1]
                goal.pose.orientation.z = q[2]
                goal.pose.orientation.w = q[3]

                #rospy.loginfo("Publishing goal: linear.x = %.2f, angular.z = %.2f", goal.pose.position.x, goal.pose.position.y)
                self.pub.publish(goal)
            else:
                rospy.logwarn("No person detected to follow.")
            self.rate.sleep()



if __name__ == '__main__':
    try:
        FollowHuman = FollowHuman()
        FollowHuman.run()
    except rospy.ROSInterruptException:
        pass
    