#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist

class Driver():
    def __init__(self):
        self.state = "idle"
        self.twist = Twist()
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)


    def cont_rotate_in_place(self, speed: float = 0.0):
        print(f"{speed=:.1f}")
        self.twist.linear.x = 0
        self.twist.angular.z = speed
        self.state = "rotate_in_place"

    def stop(self):
        self.twist.linear.x = 0
        self.twist.angular.z = 0

    def loop(self):
        print("loop driver")
        self.cmd_vel_pub.publish(self.twist)
