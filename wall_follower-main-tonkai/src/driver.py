#!/usr/bin/env python


#This node drives the robot based on information from the other nodes.

import rospy
from state_definitions import *
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16, Float32
from constants import *

#Makes the state message global
def cb_state(msg):
    global state
    state = msg.data

#Makes the twist object sent from PID global
def cb_twist(msg):
    global t_pid
    t_pid = msg

#Init node
rospy.init_node('driver')

#Make publisher for cmd_vel
pub_vel = rospy.Publisher('cmd_vel', Twist, queue_size = 1)

#Make all subscribers
sub_state = rospy.Subscriber('state', Int16, cb_state)
sub_pid_twist = rospy.Subscriber('pid_twist', Twist, cb_twist)

#Rate object
rate = rospy.Rate(10)

state = WAITING

#Create two twist variable, one is modified here, one is copied from the PID messages
t_pub = Twist()
t_pid = Twist()

print("STARTING")

while not rospy.is_shutdown():
    print("STATE: ", state)
    if (state == FOLLOWING):
        t_pub.linear.x = t_pid.linear.x
        t_pub.angular.z = t_pid.angular.z
    elif (state == RIGHT_TURN):
        # turn right
        t_pub.angular.z = 0 - ANGULAR_SPEED
    elif (state == LEFT_TURN):
        t_pub.angular.z = ANGULAR_SPEED
    elif (state == DEAD_END):
        # turn until the front is not a wall and state turns to FOLLOWING
        t_pub.angular.z = 0 - ANGULAR_SPEED
    else:
        print("STATE NOT FOUND", state)
    pub_vel.publish(t_pub)
    rate.sleep()
