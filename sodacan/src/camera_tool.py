import cv2

cap = cv2.VideoCapture(0) 

# Get range of exposure times supported 
auto_exposure = cap.get(cv2.CAP_PROP_AUTO_EXPOSURE)  
exposure = cap.get(cv2.CAP_PROP_EXPOSURE) 
print(f"{auto_exposure=} {exposure=}")
cap.set(cv2.CAP_PROP_EXPOSURE, 10000)
auto_exposure = cap.get(cv2.CAP_PROP_AUTO_EXPOSURE)  
exposure = cap.get(cv2.CAP_PROP_EXPOSURE) 
print(f"{auto_exposure=} {exposure=}")



cap.release()