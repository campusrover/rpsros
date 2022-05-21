#!/usr/bin/env python3

import rospy
import sys
from rpsexamples.msg import Sensor
import signal
import pid
from math import pi, sqrt, atan2
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from numpy import arange
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped


FORWARD_SPEED = 0.3
SLOW_FORWARD = 0.1
ROTATION = 0.8
DEBUG = True
DISTANCE = 1

class Roamer:
    def __init__(self):
        # Initialize this program as a node
        rospy.init_node("roamer")
        signal.signal(signal.SIGINT, self.shutdown)
        self.path = Path()
        self.twist = Twist()
        self.twist.linear.x = FORWARD_SPEED
        self.pid = pid.PID(-0.8, 0.5, 0.5, 0.0, 0.0)
        self.state = "start"
        self.initial_yaw = None
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        # self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odom_cb)
        self.path_pub = rospy.Publisher('/path', Path, queue_size=10)
        self.smart_lidar_sub = rospy.Subscriber("/sensor", Sensor, self.smart_lidar_cb)

    def radians_normalize(self, angle):
        """Make sure any radian angle is 0 < angle < 2*PI"""
        while angle < 0:
            angle += 2 * pi
        while angle > 2 * pi:
            angle -= 2 * pi
        return angle

    def radians_normalize2(self, angle):
        while angle < -pi:
            angle = pi - angle
        while angle > pi:
            angle -= pi
        return angle

    def radians_norm(self, angle):
        while angle < -pi:
            angle = 2*pi + angle
        while angle > pi:
            angle = angle - 2*pi
        return angle

    def kinematics(self, current_z, speed, turn):
        """
        speed is in meters/second
        turn is in radians, positive = counter clockwise
        """
        t = Twist()
        t.linear.x = speed
        t.angular.z = turn - current_z
        return t

    def distance_from(self, x1, y1, x2, y2):
        return sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)

    def pose(self, msg):
        """extract and convert pose components we need"""
        opoint = msg.pose.pose.position
        y = opoint.y
        x = opoint.x
        oquat = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([oquat.x, oquat.y, oquat.z, oquat.w])
        # yaw = atan2(2.0 * (oquat.w * oquat.z + oquat.x * oquat.y), oquat.w * oquat.w + oquat.x * oquat.x - oquat.y * oquat.y - oquat.z * oquat.z);
        return x, y, yaw

    def smart_lidar_cb(self, msg):
        # if not within 20cm of a pole, stop
        # if within 20cm then align so that pole is 9'oclock
        # move maintaining 20cm distance.
        if self.state == "start":
            if msg.shortest > 0.5 and msg.shortest < 0.1:
                self.state = "stop"
            else:
                self.state = "align"
        elif self.state == "stop":
            self.twist = Twist()
        elif self.state == "align":
            self.twist = Twist()
            self.twist.angular.z = ROTATION
            if (30 < msg.shortest_bearing < 120):
                self.state = "circle"
        elif self.state == "circle":
            self.twist = Twist()
            self.twist.linear.x = SLOW_FORWARD
            delta_distance = msg.shortest - 0.3
            self.twist.angular.z = ROTATION * delta_distance*2
        else:
            print(f"FAIL! {self.state}")

    def full_stop(self):
        self.twist = Twist()
        self.cmd_vel_pub.publish(self.twist)

    def shutdown(self, sig, stackframe):
        print("Exit because of ^c")
        self.full_stop()
        sys.exit(0)

    def run(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown() and self.state != "stop":
            self.cmd_vel_pub.publish(self.twist)
            rate.sleep()
        self.full_stop()


# rate object gets a sleep() method which will sleep 1/10 seconds
r = Roamer()
r.run()
print("*****")

# for f in arange(-6*pi, 6*pi, pi/4.0):
#     print(f"{f/2.0:2.4}, {r.radians_norm(f/2.0):1.4}")
