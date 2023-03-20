import cv2
import numpy as np
from helpers import resize_image, detectBall

cap = cv2.VideoCapture('refactoring_cv/examples/example1.avi')
 
# Check if camera opened successfully
if (cap.isOpened()== False): 
  print("Error opening video stream or file")

# Variables
n_arucos = 2
arucosCenter = [[] for _ in range(n_arucos)]
top_left_corner = 0
bot_right_corner = 0

while(cap.isOpened()):
  ret, frame = cap.read()
  if ret == True:
 
    frame = resize_image(frame, 60)
    h_frame, w_frame, _ = frame.shape
    cv2.imshow('Original',frame)

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
    parameters =  cv2.aruco.DetectorParameters_create()
    corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
    frame_markers = cv2.aruco.drawDetectedMarkers(frame.copy(), corners, ids)
    corners = np.array(corners, dtype=int)
    
    if len(corners) == n_arucos:
        top_left_corner = corners[1][0][2]
        bot_right_corner = corners[0][0][0]
        for i, corner in enumerate(corners):
            arucosCenter[i] = (int(np.mean(corner[0][:, 0])), int(np.mean(corner[0][:, 1])))
    
    _, _, x, y, radius = detectBall(frame)

    ###########
    # Drawing #
    ###########
    
    output_image = np.zeros((h_frame, w_frame, 3))
    # for i in range(n_arucos):
    #     image = cv2.circle(frame_markers, arucosCenter[i], radius=4, color=(255, 0, 255), thickness=-1)
    output_image = cv2.circle(output_image, (int(x), int(y)), radius=int(radius), color=(0, 255, 0), thickness=2)
    output_image = cv2.circle(output_image, top_left_corner, radius=4, color=(0, 255, 0), thickness=-1)
    output_image = cv2.circle(output_image, bot_right_corner, radius=4, color=(0, 0, 255), thickness=-1)
    output_image = cv2.rectangle(output_image, top_left_corner, bot_right_corner, color=(200, 200, 200), thickness=1)
    cv2.putText(output_image, "Robotic pong", (int(w_frame/2) - 70, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    cv2.imshow('Processed',output_image)
    # combined = np.where(alpha==(0,0,0), back, front)
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