import cv2
import numpy as np

cap = cv2.VideoCapture('./tests_for_components/improving_cv_speed/examples/example1.avi')
 
# Check if camera opened successfully
if (cap.isOpened()== False): 
  print("Error opening video stream or file")

while(cap.isOpened()):
  ret, frame = cap.read()
  if ret == True:
 
    cv2.imshow('Original',frame)

    # Press Q on keyboard to  exit
    if cv2.waitKey(25) & 0xFF == ord('q'):
      break
 
  # Break the loop
  else: 
    break
 
# When everything done, release the video capture object
cap.release()
 
# Closes all the frames
cv2.destroyAllWindows()


