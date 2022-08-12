#!/usr/bin/env python3

import rospy
from basenode2 import BaseNode
from rpsexamples.msg import Sensor
import bru_utils as bu
from geometry_msgs.msg import Twist
from math import pi, radians



class WanderPlatform(BaseNode):
    def initial_setup(self):
        super().initial_setup()
        self.state = "forward"
        self.shortest = None
        self.smart_lidar_sub = rospy.Subscriber("/sensor", Sensor, self.smart_lidar_cb)
        self.movement_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        while self.shortest is None:
            self.rate.sleep()

    def log(self, label: str):
        bu.info(f"{label}")

    def smart_lidar_cb(self, msg):
        self.shortest_bearing = bu.normalize_angle(radians(msg.shortest_bearing))
        self.shortest = msg.shortest

    def forward(self):
        twist = Twist()
        twist.linear.x = 0.2
        twist.angular.z = 0.0
        self.movement_pub.publish(twist)
    
    def turn(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.3
        self.movement_pub.publish(twist)
    
    def update_state(self):
        self.log(f"{self.shortest:1.2} {self.shortest_bearing:1.2}")
        if 0.3 < self.shortest < 0.6 and abs(self.shortest_bearing) < (pi / 3):
            self.state = "turn"
        else:
            self.state = "forward"


    def loop(self):
        super().loop()
        self.update_state()
        if self.state == "forward":
            self.forward()
        elif self.state == "turn":
            self.turn()
        else:
            self.shutdown_requested = True


if __name__ == "__main__":
    rospy.init_node("wanderplatform")
    rn = WanderPlatform()
    rospy.on_shutdown(rn.shutdown_hook)
    rn.run()
