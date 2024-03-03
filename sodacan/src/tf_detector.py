import rospy
import tf2_ros
from tf.transformations import euler_from_quaternion
from std_msgs.msg import Float64MultiArray

if __name__ == '__main__':

    rospy.init_node('relative_pose_calculator')

    aruco_message = Float64MultiArray()

    # Initialize TF listener
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    # Define your frame names
    camera_frame = 'camera_link'
    fiducial_frame = 'fiducial_0'
    aruco_pub = rospy.Publisher("/aruco", Float64MultiArray, queue_size=1)
    rospy.loginfo("Detector running...")

    rate = rospy.Rate(10)  # Update rate at 10 Hz

    while not rospy.is_shutdown():
        try:
            # Get the latest transform between the frames
            transform = tf_buffer.lookup_transform(camera_frame, fiducial_frame, rospy.Time(0))

            # Extract translation (x, y, z)
            translation = transform.transform.translation
            x = translation.x
            y = translation.y
            z = translation.z

            # Extract rotation (quaternion)
            rotation = transform.transform.rotation
            quaternion = (rotation.x, rotation.y, rotation.z, rotation.w)

            # Convert quaternion to Euler angles (roll, pitch, yaw)
            roll, pitch, yaw = euler_from_quaternion(quaternion)

            # Print the results
            rospy.loginfo_throttle(2, f"""{x=:1.2f} {y=:1.2f} {z=:1.2f} {roll=:1.2f} {pitch=:1.2f} {yaw=:1.2f}""")
            aruco_message.data = [x, y, z, roll, pitch, yaw]
            aruco_pub.publish(aruco_message)


        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logdebug("Error getting TF data. Retrying...")

        rate.sleep()

