#!/usr/bin/env python

import rospy
import tf2_ros
import time
import yaml

rospy.init_node('ipython')

tf_buffer = tf2_ros.Buffer()
tf_listener = tf2_ros.TransformListener(tf_buffer)
time.sleep(5.0)

frames_dict = yaml.safe_load(tf_buffer.all_frames_as_yaml())
frames_list = list(frames_dict.keys())
print(frames_list)