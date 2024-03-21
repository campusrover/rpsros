import rospy
import tf
from geometry_msgs.msg import PointStamped

if __name__ == '__main__':
    rospy.init_node('coordinate_converter')

    target = "map"
    source = "fiducial_0"
    listener = tf.TransformListener()

    # Wait for the transformation to become available
    listener.waitForTransform(target, source, rospy.Time(), rospy.Duration(1.0))

    # Specify the time at which you want to look up the transform
    transform_time = rospy.Time(0)

    try:
        # Create a PointStamped message with the source frame and coordinate
        source_point = PointStamped()
        source_point.header.frame_id = source
        source_point.point.x = 1.0
        source_point.point.y = 2.0
        source_point.point.z = 3.0

        # Transform the coordinate to the target frame
        target_point = listener.transformPoint(target, source_point)

        rospy.loginfo("Converted coordinate: %s", target_point)

    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        rospy.logwarn("Failed to convert coordinate")
