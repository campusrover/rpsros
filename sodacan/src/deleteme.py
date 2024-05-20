import cv2

try:
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_MIP_36h12)
    print("DICT_ARUCO_MIP_36h12 is supported.")
except AttributeError as e:
    print("DICT_ARUCO_MIP_36h12 is not supported:", str(e))