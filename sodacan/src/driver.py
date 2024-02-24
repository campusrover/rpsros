#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist

class Driver():
    def __init__(self):
        self._state = "idle"
        self.twist = Twist()
        self.speed = 0
        self.set_counts(0, 0, 0, 0)
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

    def rotate_in_place(self, speed: float = 0.0, moving_ticks: int = 0, waiting_ticks: int = 0):
        self.speed = speed
        self.set_counts(moving_ticks, waiting_ticks, moving_ticks, 0)
        self._state = "rotate_in_place"

    def line_patrol(self, speed: float = 0.0, moving_ticks: int = 0, waiting_ticks: int = 0):
        self.speed = speed
        self.set_counts(moving_ticks, waiting_ticks, moving_ticks, 0)
        self._state = "line_patrol"

    def stop(self):
        self._state = "stopped"
        self.twist.linear.x = 0
        self.twist.angular.z = 0

    def move(self, linear: float, angular: float):
        self._state = "move"
        self.twist.linear.x = linear
        self.twist.angular.z = angular

    def set_counts(self, moving_ticks: int, waiting_ticks: int, moving_count: int, waiting_count: int):
        self.moving_ticks = moving_ticks
        self.waiting_ticks = waiting_ticks
        self.moving_count = moving_count
        self.waiting_count = waiting_count

    def loop(self):
        rospy.loginfo(f"driver: {self._state} mov:{self.moving_count} wait:{self.waiting_count} x:{self.twist.linear.x:0.2f} z:{self.twist.angular.z:0.2f}")
        self.cmd_vel_pub.publish(self.twist)
        if self.moving_count > 0 and self.waiting_count <= 0:
            if self._state == "rotate_in_place":
                self.twist.linear.x = 0
                self.twist.angular.z = self.speed      
            elif self._state == "line_patrol": 
                self.twist.linear.x = self.speed
                self.twist.angular.z = 0
            self.moving_count -= 1
            if (self.moving_count == 0):
                self.waiting_count = self.waiting_ticks
        elif self.moving_count <= 0 and self.waiting_count > 0:
            self.twist.linear.x = 0
            self.twist.angular.z = 0          
            self.waiting_count -= 1
            if (self.waiting_count == 0):
                self.moving_count = self.moving_ticks
        elif self._state == "stopped" or self._state == "move":
            pass
        else:
            rospy.logerr(f"Driver error invalid state {self._state=}")
