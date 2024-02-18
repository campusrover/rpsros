import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class ArucoMarkerDetector:
    """
    A class to detect ArUco markers using OpenCV in a ROS environment.

    Attributes:
    - bridge (CvBridge): A bridge to convert ROS Image messages to OpenCV images.
    - marker_size (float): The size of the ArUco marker in meters.
    - dictionary (cv2.aruco.Dictionary): The ArUco marker dictionary.
    - parameters (cv2.aruco.DetectorParameters): The parameters for marker detection.
    """

    def __init__(self, marker_size: float):
        """
        Constructs a new ArucoMarkerDetector instance.

        Parameters:
        - marker_size (float): The size of the ArUco marker in meters.
        """

        # Initialize the ROS node
        rospy.init_node('aruco_marker_detector')

        # Create a CvBridge object to convert ROS Image messages to OpenCV images
        self.bridge = CvBridge()

        # Set the marker size
        self.marker_size = marker_size

        # Load the ArUco marker dictionary
        self.dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)

        # Set the parameters for marker detection
        self.parameters = cv2.aruco.DetectorParameters_create()

    def image_callback(self, msg: Image):
        """
        Callback function for the image topic.

        This function is called whenever a new image is received from the camera.

        Parameters:
        - msg (sensor_msgs.msg.Image): The ROS Image message containing the camera image.
        """

        # Convert the ROS Image message to an OpenCV image
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Detect ArUco markers in the image
        corners, ids, rejected = cv2.aruco.detectMarkers(cv_image, self.dictionary, parameters=self.parameters)

        # Draw detected markers on the image
        cv_image = cv2.aruco.drawDetectedMarkers(cv_image, corners, ids)

        # Display the image
        cv2.imshow('Aruco Marker Detection', cv_image)
        cv2.waitKey(1)

    def run(self):
        """
        Runs the ArucoMarkerDetector.

        This function starts the ROS image subscriber and waits for new images.
        It calls the image_callback function for each received image.
        """

        # Create a subscriber for the camera image topic
        rospy.Subscriber('/camera/image_raw', Image, self.image_callback)

        # Start the ROS image subscriber
        rospy.spin()

# Example usage of the ArucoMarkerDetector class:

# Create an instance of the ArucoMarkerDetector with a marker size of 0.1 meters
marker_detector = ArucoMarkerDetector(marker_size=0.1)

# Run the ArucoMarkerDetector
marker_detector.run()