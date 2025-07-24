#!/usr/bin/env python
# Simple script that publishes random 2D goals in the /cmd_vel topic (geometry_msgs/Twist)

import rospy
from geometry_msgs.msg import Twist, Image
import random

class RandomGoalGenerator:
    def __init__(self):
        # Initialize ROS node only if it hasn't been initialized yet
        rospy.init_node('point_cloud_vector', anonymous=True)
        self.sub = rospy.Subscriber( )
        self.pub = rospy.Publisher('/social_nav/global_goal', Twist, queue_size=10)
        self.rate = rospy.Rate(0.2)  # 0.2 Hz = 5 seconds interval
        
    def callback(self, data):
    	print("got image")

    def generate_random_goal(self):
        twist = Twist()
        twist.linear.x = random.uniform(-0.5, 0.5)  # Random linear velocity
        twist.angular.z = random.uniform(-0.5, 0.5)  # Random angular velocity
        return twist

    def run(self):
        while not rospy.is_shutdown():
            goal = self.generate_random_goal()
            rospy.loginfo("Publishing random goal: linear.x = %.2f, angular.z = %.2f", goal.linear.x, goal.angular.z)
            self.pub.publish(goal)
            self.rate.sleep()

if __name__ == '__main__':
    try:/spot/depth/back/image
        random_goal_generator = RandomGoalGenerator()
        random_goal_generator.run()
    except rospy.ROSInterruptException:
        pass

    
