import cv2

def find_aruco_dictionary_code():
    # Generate a list of all aruco dictionary names in the cv2.aruco module
    aruco_dict_names = [name for name in dir(cv2.aruco) if 'DICT' in name]

    # Print the name and corresponding integer value of each dictionary
    for name in aruco_dict_names:
        value = getattr(cv2.aruco, name)
        print(f"{name}: {value}")

    # Check if DICT_ARUCO_MIP_36h12 is in the list and print its value
    dict_name = 'DICT_ARUCO_MIP_36h12'
    if dict_name in aruco_dict_names:
        print(f"\nThe code for {dict_name} is {getattr(cv2.aruco, dict_name)}")
    else:
        print(f"\n{dict_name} is not available in your OpenCV version.")

if __name__ == "__main__":
    find_aruco_dictionary_code()