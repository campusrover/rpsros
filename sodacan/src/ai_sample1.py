#!/usr/bin/env python

import rospy
import tf

if __name__ == '__main__':
    rospy.init_node('coordinate_converter')

    listener = tf.TransformListener()

    # Wait for the transformation to become available
    listener.waitForTransform('/fiducial_0', '/map', rospy.Time(), rospy.Duration(1.0))

    # Specify the time at which you want to look up the transform
    transform_time = rospy.Time(0)

    # Specify the source and target frames
    source_frame = '/fiducial_0'
    target_frame = '/map'

    try:
        # Get the transform from source_frame to target_frame
        (trans, rot) = listener.lookupTransform(target_frame, source_frame, transform_time)

        # Specify the coordinate you want to convert
        source_point = (1.0, 2.0, 3.0)

        # Apply the transformation to convert the coordinate
        target_point = listener.transformPoint(target_frame, source_point, transform_time)

        rospy.loginfo("Converted coordinate: %s", target_point)

    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        rospy.logwarn("Failed to convert coordinate")

    rospy.spin()