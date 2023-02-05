#!/usr/bin/env python
import rospy
import sys
import signal
import threading
import random
from rosgraph_msgs.msg import Clock
from nav_msgs.msg import Odometry

global_flag = 0

def kill_time(power):
    dumb_array = [random.randint(1,1000) for x in range(10**power)].sort

def odom_cb(msg):
    global global_flag
    local_flag = global_flag+1
    kill_time(1)
    print(f'odom_cb: L/G Flag: {local_flag}/{global_flag}, thread: {threading.get_ident()}')
    global_flag = local_flag-1
    kill_time(1)

def clock_cb(msg):
    global global_flag
    local_flag = global_flag+1
    kill_time(6)
    print(f'clockcb: L/G Flag: {local_flag}/{global_flag}, thread: {threading.get_ident()}')
    global_flag = local_flag-1
    kill_time(1)

# Initialize this program as a node
rospy.init_node('mini_odom_demo')
odom_sub = rospy.Subscriber('/odom', Odometry, odom_cb)
clock_sub = rospy.Subscriber('/clock', Clock, clock_cb)

rospy.spin()
