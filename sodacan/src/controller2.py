#!/usr/bin/env python3

from xmlrpc.client import Boolean
import rospy
from basenode3 import BaseNode
from std_msgs.msg import Float64MultiArray
from driver import Driver

TARGET_DISTANCE = 0.26
TARGET_BEARING = 0.27

TARGET_LOST_CUTOFF = 9

LOOKING_ROTATE_SPEED = 0.9
LOOKING_TURNING_TICKS = 3
LOOKING_WAITING_TICKS = 4

INNER_CIRCLE = 0.75
OUTER_CIRCLE = 1.25

INNER_BEARING = 0.5
OUTER_BEARING = 1.0

from enum import Enum

class Rings(Enum):
    OUTER = 1
    INNER = 2
    ANY - 3

class Bearings(Enum):
    CENTER = 1
    LEFT = 2
    RIGHT = 3
    ANY = 4

class Controller:
    def __init__(self):
        super().__init__()
        self.driver = Driver()
        self.aruco_sub = rospy.Subscriber('/aruco', Float64MultiArray, self.aruco_cb)
        self.driver.rotate_in_place(LOOKING_ROTATE_SPEED, LOOKING_TURNING_TICKS, LOOKING_WAITING_TICKS)
        self.state = "looking"
        self.time_since_target = 0

    def aruco_cb(self, msg: Float64MultiArray):
        self.time_since_target = 0
        self.distance = msg.data[0]
        self.bearing = msg.data[1]
        rospy.loginfo_throttle(3, f"controller: {self.state} {self.distance:.2f} {self.bearing:.2f}")
        if self.outside_to_target_distance():
            self.state = "to target"
            self.driver.move(APPROACH_LINEAR_SPEED, -self.bearing)
        elif (not self.outside_to_target_distance() and self.state == "to target" and self.at_target_bearing()):
            self.state = "final"
            self.driver.move(FINAL_APPROACH_LINEAR_SPEED, -self.bearing)
        elif (not self.outside_to_target_distance) and self.state == "to target" and not (self.at_target_bearing()):
            self.state = "escape"
            self.drive.move(-APPROACH_LINEAR_SPEED, 0)
        elif self.at_target_distance() and self.at_target_bearing():
            self.state = "at target"
            self.driver.stop()
        elif self.state == "stop" or self.state == "looking":
            self.driver.stop()
        elif self.state == "escape":
            pass
        else:
            rospy.logerr(f"Invalid state in controller.py {self.state=} {self.bearing=} {self.distance=}")
            self.state = "stop"
            self.driver.stop()
    
    def in_inner_circle_center_inner_bearing(self):
        pass
    
    def in_outer_circle_center_outer_bearing(self):
        pass

    def inner_circle_inner_left_bearing(self):
        pass

    def inner_circle_inner_right_bearing(self):
        pass

    def in_zone(bearing: Bearings, ring: Rings):



   def run(self):
        self.rate = rospy.Rate(30)
        try:
            while not rospy.is_shutdown():
                self.time_since_target += 1
                if (self.state != "looking" and self.time_since_target > TARGET_LOST_CUTOFF):
                    rospy.loginfo("lost sight of target.")
                    self.state = "stop"
                self.driver.loop()
                self.rate.sleep()

        except rospy.exceptions.ROSInterruptException:
            rospy.loginfo("exiting...")

if __name__ == "__main__":
    rospy.init_node("Controller")
    controller = Controller()
    rospy.loginfo("Controller Started...")
    controller.run()
