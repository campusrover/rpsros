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

    def rotate_in_place(self, speed: float = 0.0, turning_ticks: int = 0, waiting_ticks: int = 0):
        self.speed = speed
        self.set_counts(turning_ticks, waiting_ticks, turning_ticks, 0)
        self._state = "rotate_in_place"

    def line_patrol(self, speed: float = 0.0, turning_ticks: int = 0, waiting_ticks: int = 0):
        self.speed = speed
        self.set_counts(turning_ticks, waiting_ticks, turning_ticks, 0)
        self._state = "line_patrol"

    def stop(self):
        self._state = "stopped"
        self.twist.linear.x = 0
        self.twist.angular.z = 0

    def move(self, linear: float, angular: float):
        self.state = "move"
        self.twist.linear.x = linear
        self.twist.angular.z = angular

    def set_counts(self, turning_ticks: int, waiting_ticks: int, turning_count: int, waiting_count: int):
        self.turning_ticks = turning_ticks
        self.waiting_ticks = waiting_ticks
        self.turning_count = turning_count
        self.waiting_count = waiting_count

    def loop(self):
        rospy.loginfo(f"{self._state} {self.turning_ticks=} {self.waiting_ticks=}")
        self.cmd_vel_pub.publish(self.twist)
        if self._state == "rotate_in_place" and self.turning_count > 0 and self.waiting_count <= 0:
            self.twist.linear.x = 0
            self.twist.angular.z = self.speed        
            self.turning_ticks -= 1
            if (self.turning_ticks == 0):
                self.passive_count = self.waiting_count
        elif self._state == "rotate_in_place" and self.turning_count <= 0 and self.waiting_count >= 0:
            self.twist.linear.x = 0
            self.twist.angular.z = 0          
            self.passive_count -= 1
            if (self.waiting_ticks == 0):
                self.turning_ticks = self.active_ticks
        elif self._state == "line_patrol" and self.turning_count > 0 and self.waiting_count <= 0:
            self.twist.linear.x = self.speed
            self.twist.angular.z = 0
            self.turning_ticks -= 1
            if (self.turning_ticks == 0):
                self.passive_count = self.waiting_count
        elif self._state == "line_patrol" and self.turning_count <= 0 and self.waiting_count >= 0:
            self.twist.linear.x = -self.speed
            self.twist.angular.z = 0          
            self.passive_count -= 1
            if (self.passive_count == 0):
                self.active_count = self.active_ticks
        elif self._state == "stopped" or self._state == "move":
            pass
        else:
            rospy.logerr(f"Driver error invalid state {self._state=}")
