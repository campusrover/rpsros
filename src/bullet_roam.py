#!/usr/bin/env python3

import rospy
import sys
import signal
import pid
from math import pi, sqrt
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler

FORWARD_SPEED = 0.3
ROTATION = 0.5
DEBUG = True

class Roamer:
    def __init__(self):
        # Initialize this program as a node
        rospy.init_node("outback")
        signal.signal(signal.SIGINT, self.shutdown)
        self.twist = Twist()
        # self.twist.linear.x = FORWARD_SPEED
        self.pid = pid.PID(-0.5, 0.5, 0.5, 0.0, 0.0)
        self.state = "idle"
        self.initial_yaw = None
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odom_cb)
        if DEBUG:
            print("dir, dist, yaw, bearingerror")

    def print_debug(self, arg):
        if DEBUG:
            print(
                f"{arg},{self.dist_from_start:2.3},{self.yaw:2.3},{self.bearing_error:2.3}"
            )

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
        while angle < -pi/2.0:
            angle = pi + angle
        while angle > pi/2.0:
            angle = angle - pi
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

    def distance_from_start(self, x, y):
        return sqrt((x - self.start_x) ** 2 + (y - self.start_y) ** 2)

    def pose(self, msg):
        """extract and convert pose components we need"""
        opoint = msg.pose.pose.position
        y = opoint.y
        x = opoint.x
        oquat = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([oquat.x, oquat.y, oquat.z, oquat.w])
        # if DEBUG: print(f"x: {x:1.2} y:{y:1.2} yw:{yaw:1.2}")
        return x, y, self.radians_norm(yaw)

    def odom_cb(self, msg):
        self.x, self.y, self.yaw = self.pose(msg)
        if self.initial_yaw == None:
            self.initial_yaw = self.yaw
            self.start_x = self.x
            self.start_y = self.y
            self.target_yaw = self.yaw
        self.dist_from_start = self.distance_from_start(self.x, self.y)
        self.bearing_error = self.radians_norm(self.yaw - self.target_yaw)
        if self.state == "start":
            if self.dist_from_start >= 1.0:
                self.state = "turn"
                self.target_yaw = self.radians_norm(self.yaw + pi)
                print(f"sy: {self.yaw:1.2} ty: {self.target_yaw:1.2}")
            pid_turn = self.pid.compute(self.target_yaw, self.yaw)
            self.twist.angular.z = pid_turn
            self.print_debug("start")
        elif self.state == "turn":
            rotation_from_target = self.radians_norm(self.yaw - self.target_yaw)
            self.dist_from_start = self.distance_from_start(self.x, self.y)

            if -0.05 < rotation_from_target < 0.05:
                self.state = "return"
                self.target_yaw = self.radians_norm(self.initial_yaw + pi)
                print(f"ty: {self.target_yaw:1.2} iy: {self.initial_yaw:1.2}")
            self.twist.angular.z = ROTATION
            self.twist.linear.x = 0
            self.print_debug("turn")
        elif self.state == "return":
            self.twist.linear.x = FORWARD_SPEED
            self.dist_from_start = self.distance_from_start(self.x, self.y)
            # self.bearing_error = self.radians_norm(yaw - self.target_yaw)
            if -0.25 < self.dist_from_start < 0.25 or self.dist_from_start > 2:
                self.state = "stop"
            pid_turn = self.pid.compute(self.target_yaw, self.yaw)
            self.twist.angular.z = pid_turn
            self.print_debug("return")
        elif self.state == "idle":
            self.print_debug("idle")
        else:
            print("FAIL! {self.state}")

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

# for f in range(-int(30.0*pi),+int(30.0*pi)):
#     print(f"{f/10.0:1.3}, {r.radians_norm(f/10.0):1.3}")
