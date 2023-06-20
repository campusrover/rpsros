#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String

class RoboGym:
    def __init__(self):
        self.pub_cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_cb)
        self.target = Point(1.0,0,0)
    
    def odom_cb(self, msg):
        self.oreuler = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([oreuler.x, oreuler.y, oreuler.z, oreuler.w])
        self.yaw = radians_normalize(yaw)
        self.position = msg.pose.pose.position


# Main function.
if __name__ == "__main__":
    # Initialize the node and name it.
    rospy.init_node("robogym")
    try:
        RoboGym().run()
    except rospy.ROSInterruptException:
        pass
