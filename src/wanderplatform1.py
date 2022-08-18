#!/usr/bin/env python3

import rospy
from basenode2 import BaseNode
from rpsexamples.msg import Sensor
import bru_utils as bu
from geometry_msgs.msg import Twist
from math import pi, radians, isnan

WEDGE_1_ANGLE = pi/3.0
FWD_RING1 = 1.0
REAR_RING1 = 0.5
RING2 = 1.5


FAST_LINEAR_SPEED = 0.2
SLOW_LINEAR_SPEED = 0.1
TURN_ANGULAR_SPEED = 0.15
SLOW_ANGULAR_SPEED = 0.1


class WanderPlatform(BaseNode):
    def initial_setup(self):
        super().initial_setup()
        self.state = ""
        self.shortest = None
        self.smart_lidar_sub = rospy.Subscriber(
            "/sensor", Sensor, self.smart_lidar_cb)
        self.movement_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        while self.shortest is None:
            self.rate.sleep()

    def log(self, label: str):
        bu.info(f"{label}")

    def smart_lidar_cb(self, msg):
        self.shortest_bearing = bu.normalize_angle(
            radians(msg.shortest_bearing))
        self.shortest = msg.shortest

    def RING2(self):
        return self.shortest > RING2

    def forward_wedge(self):
        return abs(self.shortest_bearing) < WEDGE_1_ANGLE

    def RING1(self):
        if self.forward_wedge() and self.shortest < FWD_RING1:
            return True
        elif not self.forward_wedge() and self.shortest < REAR_RING1:
            return True
        elif isnan(self.shortest_bearing):
            return True
        else:
            return False

    def RING0_FL(self):
        return 0 < self.shortest_bearing < -WEDGE_1_ANGLE and self.shortest < FWD_RING1

    def RING0_FR(self):
        return 0 > self.shortest_bearing > -WEDGE_1_ANGLE and self.shortest < FWD_RING1

    def RING0_RR(self):
        return -WEDGE_1_ANGLE < self.shortest_bearing < -pi and self.shortest < REAR_RING1

    def RING0_RL(self):
        return WEDGE_1_ANGLE < self.shortest_bearing < pi and self.shortest < REAR_RING1

    def update_state(self, twist: Twist):
        self.log(
            f"STATE: {self.state}: short: {self.shortest:1.2} bearing: {self.shortest_bearing:1.2}")
        if self.RING0_RR():
            self.state = "RING0_RR"
            twist.linear.x = SLOW_LINEAR_SPEED
            twist.angular.z = -SLOW_ANGULAR_SPEED
        elif self.RING0_RL():
            self.state = "RING0_RL"
            twist.linear.x = SLOW_LINEAR_SPEED
            twist.angular.z = SLOW_ANGULAR_SPEED
        elif self.RING0_FL():
            self.state = "RING0_FL"
            twist.linear.x = -SLOW_LINEAR_SPEED
            twist.angular.z = SLOW_ANGULAR_SPEED
        elif self.RING0_FR():
            self.state = "RING0_FR"
            twist.linear.x = -SLOW_LINEAR_SPEED
            twist.angular.z = -SLOW_ANGULAR_SPEED
        elif self.RING1():
            self.state = "RING1"
            twist.linear.x = SLOW_LINEAR_SPEED
            twist.angular.z = 0
        elif self.RING2():
            state = "RING2"
            twist.linear.x = FAST_LINEAR_SPEED
            twist.angular.z = 0
        else:
            self.state = "error"
            #self.shutdown_requested = True

    def loop(self):
        super().loop()
        twist = Twist()
        self.update_state(twist)
        #self.movement_pub.publish(twist)


if __name__ == "__main__":
    rospy.init_node("wanderplatform")
    rn = WanderPlatform()
    rospy.on_shutdown(rn.shutdown_hook)
    rn.run()
