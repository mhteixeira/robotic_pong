import cv2
import numpy as np
from params import *
from helpers import resize_image
import cv2.aruco as aruco

# Create a VideoCapture object and read from input file
# If the input is the camera, pass 0 instead of the video file name
greenLower = (20, 80, 6)
greenUpper = (64, 255, 255)

cap = cv2.VideoCapture('refactoring_cv/examples/example3.avi')

# Check if camera opened successfully
if (cap.isOpened()== False): 
  print("Error opening video stream or file")

def detectBall(frame):
  frame = frame.copy()
  blurred = cv2.GaussianBlur(frame, (11, 11), 0)
  hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

  # construct a mask for the color "green", then perform
  # a series of dilations and erosions to remove any small
  # blobs left in the mask

  mask = cv2.inRange(hsv, greenLower, greenUpper)
  mask = cv2.erode(mask, None, iterations=2)
  mask = cv2.dilate(mask, None, iterations=2)
  frame_cnts, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
  # cv2.drawContours(frame, frame_cnts], -1, (0,255,0), 3)
  if len(frame_cnts) > 0:
    c = max(frame_cnts, key=cv2.contourArea)
    ((x, y), radius) = cv2.minEnclosingCircle(c)
    diameter = 2*radius
    M = cv2.moments(c)
    center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
    cv2.circle(frame, (int(x), int(y)), int(radius), (0, 0, 255), 2)
  return frame, frame_cnts

# Read until video is completed
while(cap.isOpened()):
  ret, frame = cap.read()
  if ret == True:
 
    frame = resize_image(frame, 60)
    processed_frame, _ = detectBall(frame)

    print(arUcodetector(frame))
    cv2.imshow('Original',frame)
    cv2.imshow('Processed',processed_frame)


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


