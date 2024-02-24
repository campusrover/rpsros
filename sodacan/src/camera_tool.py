import cv2

cap = cv2.VideoCapture(0) 

# Get range of exposure times supported 
min_exposure = cap.get(cv2.CAP_PROP_AUTO_EXPOSURE)  
max_exposure = cap.get(cv2.CAP_PROP_AUTO_EXPOSURE) 
print(min_exposure, max_exposure)

# Set exposure time to desired value (in microseconds)
exposure_us = 1000   # 1 ms exposure
cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, exposure_us)
cap.release()