#!/usr/bin/env python


#This node drives the robot based on information from the other nodes.

import rospy
from state_definitions import *
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16, Float32
import random

#Makes the state message global
def cb_state(msg):
    global state
    state = msg.data

#Makes the twist object sent from PID global
def cb_twist(msg):
    global t_pid
    t_pid = msg

#randomly edits the current angular speed by a small amount for wandering state
def get_rand(cur):
    #subtracting 0.5 allows for a change in angular speed between -0.5 and 0.5
    if cur > 1:
        cur = 0
    if cur < -1:
        cur = 0
    change = float(random.random())
    change = change - 0.5
    return cur + change

#Init node
rospy.init_node('driver')

#Make publisher for cmd_vel
pub_vel = rospy.Publisher('cmd_vel', Twist, queue_size = 1)

#Make all subscribers
sub_state = rospy.Subscriber('/state', Int16, cb_state)
sub_pid_twist = rospy.Subscriber('/twist', Twist, cb_twist)

#Rate object
rate = rospy.Rate(10)

state = 5

#Create two twist variable, one is modified here, one is copied from the PID messages
t_pub = Twist()
t_pid = Twist()

print("STARTING")

while not rospy.is_shutdown():
    print("STATE: ", state)
    if (state == 1):
        t_pub.angular.z = -1 * t_pid.angular.z
        t_pub.linear.x = t_pid.linear.x
    elif (state == 2):
        t_pub.angular.z = t_pid.angular.z
        t_pub.linear.x = t_pid.linear.x
    elif (state == 3):
        t_pub.angular.z = 1
        t_pub.linear.x = 0
    elif (state == 4):
        t_pub.angular.z = -1
        t_pub.linear.x = 0
    elif (state == 5):
        t_pub.angular.z = get_rand(t_pub.angular.z)
        t_pub.linear.x = t_pid.linear.x
    else:
        print("STATE NOT FOUND")
    pub_vel.publish(t_pub)
    rate.sleep()
