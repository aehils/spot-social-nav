#!/usr/bin/env python
# Simple script that susbscribes to the /social_nav_perception/people_in_odom_frame, gets the latest human pose, and sends a goal to reach that location
import rospy
from geometry_msgs.msg import Twist
import random
from social_nav_perception.msg import PersonArray, Person

class FollowHuman:
    def __init__(self):
        # Initialize ROS node only if it hasn't been initialized yet
        rospy.init_node('follow_human', anonymous=True)
        self.sub = rospy.Subscriber('/social_nav_perception/people_in_odom_frame', PersonArray, self.people_callback)
        self.pub = rospy.Publisher('/spot/cmd_vel', Twist, queue_size=10)
        self.rate = rospy.Rate(1.0) # 1 Hz = 1 second interval
        self.latest_person = None

    def run(self):
        while not rospy.is_shutdown():
            if self.latest_person:
                goal = Twist()
                goal.linear.x = self.latest_person.pose.position.x
                goal.angular.z = self.latest_person.pose.position.y
                rospy.loginfo("Publishing goal: linear.x = %.2f, angular.z = %.2f", goal.linear.x, goal.angular.z)
                self.pub.publish(goal)
            else:
                rospy.logwarn("No person detected to follow.")
            self.rate.sleep()

    def people_callback(self, msg):
        if msg.people:
            # Assuming we want to follow the first detected person
            self.latest_person = msg.people[0]
            rospy.loginfo("Detected person with ID: %d at position (%.2f, %.2f, %.2f)", 
                          self.latest_person.id, 
                          self.latest_person.pose.position.x, 
                          self.latest_person.pose.position.y, 
                          self.latest_person.pose.position.z)
        else:
            self.latest_person = None

if __name__ == '__main__':
    try:
        follow_human = FollowHuman()
        follow_human.run()
    except rospy.ROSInterruptException:
        pass
    