#!/usr/bin/env python3
""" This is an attempt at driving the robot to the fiducial and then picking it up.
I have decided not to worry about approaching the payload head on. Given the size of the robot and the size of my space,
I think that will be very difficult without the ability to move sideways which I don't have.
"""

HZ = 20
LINE_MOVE_TIME = 8
LINE_PAUSE_TIME = 0.2
LINE_SPEED = 0.1

import rospy
from geometry_msgs.msg import Twist

class Driver():
    def __init__(self):
        self.state = "idle"
        self.twist = Twist()
        self.speed = 0
        self.set_counts(0, 0, 0, 0)
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

    def rotate_in_place(self, speed: float = 0.0, moving_ticks: int = 0, waiting_ticks: int = 0):
        self.speed = speed
        self.set_counts(moving_ticks, waiting_ticks, moving_ticks, 0)
        self.state = "rotate_in_place"

    def line_patrol(self, speed: float = 0.0, moving_ticks: int = 0, waiting_ticks: int = 0):
        self.speed = speed
        self.set_counts(moving_ticks, waiting_ticks, moving_ticks, 0)
        self.state = "line_patrol"

    def stop(self):
        self.state = "stopped"
        self.twist.linear.x = 0
        self.twist.angular.z = 0

    def move(self, linear: float, angular: float):
        self.state = "move"
        self.twist.linear.x = linear
        self.twist.angular.z = angular

    def set_counts(self, moving_ticks: int, waiting_ticks: int, moving_count: int, waiting_count: int):
        self.moving_ticks = moving_ticks
        self.waiting_ticks = waiting_ticks
        self.moving_count = moving_count
        self.waiting_count = waiting_count

    def loop(self):
        rospy.loginfo_throttle(3, f"driver: {self.state} mov:{self.moving_count} wait:{self.waiting_count} x:{self.twist.linear.x:0.2f} z:{self.twist.angular.z:0.2f}")
        self.cmd_vel_pub.publish(self.twist)
        if self.moving_count > 0 and self.waiting_count <= 0:
            if self.state == "rotate_in_place":
                self.twist.linear.x = 0
                self.twist.angular.z = self.speed      
            elif self.state == "line_patrol":
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
                if self.state == "line_patrol":
                    self.speed = -self.speed
        elif self.state == "stopped" or self.state == "move":
            pass
        else:
            rospy.logerr(f"Driver error invalid state {self.state=} {self.moving_count=} {self.waiting_count}")

if __name__ == "__main__":
    rospy.init_node("driver_tester")
    driver = Driver()
    driver.line_patrol(LINE_SPEED, int(LINE_MOVE_TIME * HZ), int(LINE_PAUSE_TIME * HZ))
    rate = rospy.Rate(HZ)
    try:
        while not rospy.is_shutdown():
            driver.loop()
            rate.sleep()
    except rospy.exceptions.ROSInterruptException:
        rospy.loginfo("exiting...")