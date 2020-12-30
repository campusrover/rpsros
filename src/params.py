#!/usr/bin/env python
import rospy

# Set wallfollow parameters
rospy.set_param("wallfollow/forward_speed", 0.1)
rospy.set_param("wallfollow/desired_distance_from_wall", 0.5)
rospy.set_param("wallfollow/speed_limit", 0.2)

# set smartpid parameters
rospy.set_param("smartpid/Kp", -0.44)
rospy.set_param("smartpid/Td", -0.4)
rospy.set_param("smartpid/Ti", 0)
rospy.set_param("smartpid/dt",0.1)
