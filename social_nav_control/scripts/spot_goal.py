#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
import keyboard

class NavigationGoal:
    def __init__(self):
        rospy.init_node("nav_goal")
        self.pub = rospy.Publisher('/spot/go_to_pose', PoseStamped, queue_size=10)
        
    def send_cmd(self,command):
        twist = PoseStamped()
        twist.pose.position.x = command[0]
        twist.pose.position.y = command[1]
        twist.pose.position.z = command[2]
        twist.header.frame_id = 'body'
        self.pub.publish(twist)
        

    def key_loop(self):
        allowed_keys = ['w', 'a', 's', 'd']  # list of allowed keys
        running = 1
        while True: 
            try:
                command = 0
                letter = input("press wasd:")                
                if letter == 'q': 
                    print('Exiting...')
                    break

                elif letter.lower() in allowed_keys:
                    print(f'You pressed {letter}')
                    if letter == 'w':
                        command = (1.0, 0, 0)
                    elif letter == 'a':
                        command = (0, 1, 0)
                    elif letter == 's':
                        command = (-1, 0, 0)
                    elif letter == 'd':
                        command = (0, 1, 0)
                    elif letter == 'h':
                        command = (0, 0, 0)
                else:
                    print('You can only use W, A, S, D keys!')
                
                if command:
                    self.send_cmd(command)

        
            except Exception as e:
                print(f"An error occurred: {e}")
                break  # if an error occurs, break the loop


if __name__ == '__main__':
    try:
        navigate = NavigationGoal()
        navigate.key_loop()
    except rospy.ROSInterruptException:
        pass


    
