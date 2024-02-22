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
        self.driver.rotate_in_place(0.9, 3, 2)
        self.state = "looking"

    def aruco_cb(self, msg: Float64MultiArray):
        distance = msg.data[0]
        bearing = msg.data[1]
        rospy.loginfo(f"{distance=:.1f} {bearing=:.1f}")
        if distance > 1.0:
            self.state = "approach"
            self.driver.move(0.5, 0.0)
        elif distance < 1.0:
            self.state = "stop"
            rospy.loginfo(f"approach completed")
            self.driver.stop
        else:
            rospy.logerr("Invalid state in controller.py")
            self.driver.stop

    def loop(self):
        self.driver.loop()

if __name__ == "__main__":
    rospy.init_node("Controller")
    controller = Controller()
    rospy.on_shutdown(controller.shutdown_hook)
    controller.run()
