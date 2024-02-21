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


    def cont_rotate_in_place(self, speed: float = 0.0, active_ticks: int = 0, passive_ticks: int = 0):
        self.speed = speed
        self.set_counts(active_ticks, passive_ticks, active_ticks, 0)
        self._state = "rotate_in_place"

    def stop(self):
        self._state = "stopped"
        self.twist.linear.x = 0
        self.twist.angular.z = 0

    def state(self):
        return self.state

    def set_counts(self, active_ticks: int, passive_ticks: int, active_count: int, passive_count: int):
        self.active_ticks = active_ticks
        self.passive_ticks = passive_ticks
        self.active_count = active_count
        self.passive_count = passive_count

    def loop(self):
        rospy.logdebug(f"{self.active_count=} {self.passive_count=}")
        self.cmd_vel_pub.publish(self.twist)
        if self._state == "rotate_in_place" and self.active_count > 0 and self.passive_count <= 0:
            self.twist.linear.x = 0
            self.twist.angular.z = self.speed        
            self.active_count -= 1
            if (self.active_count == 0):
                self.passive_count = self.passive_ticks
        elif self._state == "rotate_in_place" and self.active_count <= 0 and self.passive_count >= 0:
            self.twist.linear.x = 0
            self.twist.angular.z = 0          
            self.passive_count -= 1
            if (self.passive_count == 0):
                self.active_count = self.active_ticks
        elif self._state == "stopped":
            pass
        else:
            rospy.logerror("Driver error invalid state")

