#!/usr/bin/env python

import rospy
import sys
import signal
from math import pi
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler


# # [ [ x, 0], ... ]
# waypoints = [[1,0], [1,0]]
way_index = 0

state = "aligning"
twist = Twist()
exit_loop = False

def radians_normalize(angle):
    while(angle<0):
        angle += 2*pi
    while(angle>2*pi):
        angle -= 2*pi
    print angle
    return angle

def odom_cb(msg):
    global twist
    global target_y, target_x
    global waypoints, way_index
    # target_x, target_y = waypoints[way_index]
    target_x = 2.0
    target_y = 0.0
    ppoint = msg.pose.pose.position
    y = ppoint.y; x = ppoint.x
    delta_x = x - target_x
    delta_y = y - target_y
    twist.linear.x = 0.4
    twist.angular.z = delta_y * -0.7
    print("i: %d dx: %0.2f dy: %0.2f, tx: %0.2f tz: %0.2f" % 
            (way_index, delta_x, delta_y, twist.linear.x, twist.angular.z))
    if delta_x >= 0:
        print("***********")
        way_index = 1
        
 


def shutdown(sig, stackframe):
    print("Exit because of ^c")
    twist.angular.z = 0
    twist.linear.x = 0
    cmd_vel_pub.publish(twist)
    sys.exit(0)

# Initialize this program as a node
cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
odom_sub = rospy.Subscriber('/odom', Odometry, odom_cb)
rospy.init_node('outback')
signal.signal(signal.SIGINT, shutdown)


# rate object gets a sleep() method which will sleep 1/10 seconds
rate = rospy.Rate(20)

while not rospy.is_shutdown() and way_index < 1:
    cmd_vel_pub.publish(twist)
    rate.sleep()

twist.angular.z = 0
twist.linear.x = 0
cmd_vel_pub.publish(twist)

