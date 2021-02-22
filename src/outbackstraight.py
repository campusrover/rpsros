#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler

state = "aligning"
twist = Twist()

def odom_cb(msg):
    global state
    global twist
    if state == "aligning":
        diff = euler_from_quaternion(msg.pose.pose.orientation.z)
        twist.twist.angular.z = diff
        if (-0.05 < diff < 0.05):
            state = "outgoing"
        # turn around until odom rotation is 0
    elif state == "outgoing":
        # move forward while maintaining location y of odom at 0 unchanged
        state = "turning"
    elif state == "turning":
        # turn while maintiaing location odom fixed
        state = "returning"
    elif state == "returnng":
        # move forward while maintaining location y of odom unchanged
        state = "complete"
    elif state == "complete":
        pass
    
# Initialize this program as a node
cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
odom_sub = rospy.Subscriber('odom', Odometry, odom_cb)
rospy.init_node('outback')
twist = Twist()
# rate object gets a sleep() method which will sleep 1/10 seconds
rate = rospy.Rate(10)

while not rospy.is_shutdown() and state != "complete":
    print (twist)
    cmd_vel_pub.publish(twist)
    rate.sleep()