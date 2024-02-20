#!/usr/bin/env python3

import rospy
from basenode3 import BaseNode
from std_msgs.msg import Float64MultiArray
from driver import Driver

class Controller(BaseNode):
    def __init__(self):
        super().__init__()
        self.driver = Driver()
        self.aruco_sub = rospy.Subscriber('/aruco', Float64MultiArray, self.aruco_cb)
        self.driver.cont_rotate_in_place(-0.4)

    def aruco_cb(self, msg: Float64MultiArray):
        print("callback")
        distance = msg.data[0]
        bearing = msg.data[1]
        rospy.loginfo(f"{distance=:.1f} {bearing=:.1f}")

    def loop(self):
        print("loop")
        self.driver.loop()

if __name__ == "__main__":
    print("controller start")
    rospy.init_node("Controller")
    controller = Controller()
    controller.run()
