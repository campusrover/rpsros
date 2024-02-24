#!/usr/bin/env python3

from xmlrpc.client import Boolean
import rospy
from basenode3 import BaseNode
from std_msgs.msg import Float64MultiArray
from driver import Driver

TARGET_DISTANCE = 0.215
TARGET_BEARING = 0.249
TARGET_LOST_CUTOFF = 9

LOOKING_ROTATE_SPEED = 0.9
LOOKING_TURNING_TICKS = 3
LOOKING_WAITING_TICKS = 4

APPROACH_LINEAR_SPEED = 0.3
APPROACH_DISTANCE = 1.0

FINAL_APPROACH_DISTANCE = 0.5
FINAL_APPRPACH_LINEAR_SPEED = 0.15

class Controller(BaseNode):
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
        rospy.loginfo(f"controller: {self.state} {self.distance:.2f} {self.bearing:.2f}")
        if self.outside_to_target_distance():
            self.state = "to target"
            self.driver.move(APPROACH_LINEAR_SPEED, -self.bearing)
        elif (not self.outside_to_target_distance()):
            self.state = "final"
            self.driver.move(FINAL_APPRPACH_LINEAR_SPEED, -self.bearing)
        elif self.at_target():
            self.state = "at target"
            self.driver.stop()
        elif self.state == "stop":
            self.driver.stop()
        else:
            rospy.logerr(f"Invalid state in controller.py {self.state=} {self.bearing=} {self.distance=}")
            self.state = "stop"
            self.driver.stop()

    def outside_to_target_distance(self) -> Boolean:
        return self.distance > APPROACH_DISTANCE

    def at_target(self):
        return abs(self.distance - TARGET_DISTANCE) < 0.1 and abs(self.bearing - TARGET_BEARING) < 0.1
    
    def loop(self):
        self.time_since_target += 1
        if (self.state != "looking" and self.time_since_target > TARGET_LOST_CUTOFF):
            self.state = "looking"
            self.driver.rotate_in_place(LOOKING_ROTATE_SPEED, LOOKING_TURNING_TICKS, LOOKING_WAITING_TICKS)                    
        self.driver.loop()

if __name__ == "__main__":
    rospy.init_node("Controller")
    controller = Controller()
    rospy.on_shutdown(controller.shutdown_hook)
    rospy.loginfo("Controller Started...")
    controller.run()
