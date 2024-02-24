#!/usr/bin/env python3

from xmlrpc.client import Boolean
import rospy
from basenode3 import BaseNode
from std_msgs.msg import Float64MultiArray
from driver import Driver

TARGET_DISTANCE = 0.215
TARGET_BEARING = 0.249
TARGET_LOST_CUTOFF = 10

LOOKING_ROTATE_SPEED = 0.3
LOOKING_TURNING_TICKS = 3
LOOKING_PAUSING_TICKS = 2

TO_TARGET_LINEAR_SPEED = 0.5
TO_TARGET_DISTANCE = 1.0

FINAL_APPROACH_DISTANCE = 0.5
FINAL_APPRPACH_LINEAR_SPEED = 0.1



class Controller(BaseNode):
    def __init__(self):
        super().__init__()
        self.driver = Driver()
        self.aruco_sub = rospy.Subscriber('/aruco', Float64MultiArray, self.aruco_cb)
        self.driver.rotate_in_place(LOOKING_ROTATE_SPEED, LOOKING_TURNING_TICKS, LOOKING_PAUSING_TICKS)
        self.state = "looking"
        self.time_since_target = 0
        

    def aruco_cb(self, msg: Float64MultiArray):
        self.time_since_target = 0
        self.distance = msg.data[0]
        self.bearing = msg.data[1]
        rospy.logdebug(f"{self.distance=:.1f} {self.bearing=:.1f}")
        if self.outside_target_distance() and self.state == "looking":
            self.state = "to target"
            self.driver.move(TO_TARGET_LINEAR_SPEED, -self.bearing)
        elif not self.outside_target_distance() and self.state == "to target":
            self.state = "final"
            self.driver.move(FINAL_APPRPACH_LINEAR_SPEED, -self.bearing)
        elif self.at_target():
            self.state = "at target"
            self.driver.stop()
        else:
            rospy.logerr("Invalid state in controller.py")
            self.driver.stop()

    def outside_target_distance(self) -> Boolean:
        return self.distance > TO_TARGET_DISTANCE

    def at_target(self):
        return abs(self.distance - TARGET_DISTANCE) < 0.1 and abs(self.bearing - TARGET_BEARING) < 0.1
    
    def loop(self):
        self.time_since_target += 1
        if (self.time_since_target > TARGET_LOST_CUTOFF):
            self.state = "looking"
            self.driver.rotate_in_place(0.9, 3, 2)
            
        self.driver.loop()

if __name__ == "__main__":
    rospy.init_node("Controller")
    controller = Controller()
    rospy.on_shutdown(controller.shutdown_hook)
    controller.run()
