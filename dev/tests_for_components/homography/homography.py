import cv2
import numpy as np
import pickle
from pyniryo2 import *
import time

robot = NiryoRobot("169.254.200.200")

h, w = 640, 480
cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)

# Load camera calibration parameters
file = open("./assets/calibration/calibration.pkl",'rb')
cameraMatrix, dist = pickle.load(file)

newCameraMatrix, roi = cv2.getOptimalNewCameraMatrix(cameraMatrix, dist, (w,h), 1, (w,h))
field_delimited = False
M = None
while not field_delimited:

    succes, frame = cap.read()

    h,  w = frame.shape[:2]
    newCameraMatrix, roi = cv2.getOptimalNewCameraMatrix(cameraMatrix, dist, (w,h), 1, (w,h))
    dst = cv2.undistort(frame, cameraMatrix, dist, None, newCameraMatrix)
    dst = dst[roi[1]:roi[1]+roi[3], roi[0]:roi[0]+roi[2]]

    # First we make the image monochromatic
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Next we configure the ArUco detector
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
    parameters =  cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)
    
    # Then we detect and draw the ArUcos
    corners, ids, rejectedImgPoints = detector.detectMarkers(gray)
    frame_markers = cv2.aruco.drawDetectedMarkers(frame.copy(), corners, ids)
    corners = np.array(corners, dtype=int)
    
    # We now check if the amount of detected ArUcos is the expected one
    # If it is not, the detection failed
    if len(corners) == 4:
        field_delimited = True
        # The top left corner is identified by the Aruco with ID = 0
        top_left_corner_id = np.where(ids == 0)[0]
        
        # The bottom right corner is identified by the Aruco with ID = 2
        bottom_right_corner_id = np.where(ids == 2)[0]

        # The bottom right corner is identified by the Aruco with ID = 3
        top_right_corner_id = np.where(ids == 3)[0]

        # The bottom right corner is identified by the Aruco with ID = 2
        bottom_left_corner_id = np.where(ids == 1)[0]

        top_left_corner = corners[top_left_corner_id][0][0][0]
        bottom_right_corner = corners[bottom_right_corner_id][0][0][0]
        top_right_corner = corners[top_right_corner_id][0][0][0]
        bottom_left_corner = corners[bottom_left_corner_id][0][0][0]

        width_top = np.hypot(top_left_corner[0] - top_right_corner[0], top_left_corner[1] - top_right_corner[1])
        width_bottom = np.hypot(bottom_left_corner[0] - bottom_right_corner[0], bottom_left_corner[1] - bottom_right_corner[1])
        max_width = max(int(width_top), int(width_bottom))
        height_left = np.hypot(top_left_corner[0] - bottom_left_corner[0], top_left_corner[1] - bottom_left_corner[1])
        height_right = np.hypot(top_right_corner[0] - bottom_right_corner[0], top_right_corner[1] - bottom_right_corner[1])
        max_height = max(int(height_left), int(height_right))

        input_pts = np.float32([top_left_corner, bottom_left_corner, bottom_right_corner, top_right_corner])
        output_pts = np.float32([[0, 0],
                                [0, max_height - 1],
                                [max_width - 1, max_height - 1],
                                [max_width - 1, 0]])

        pixel_density = max_height/0.64

        # Compute the perspective transform M
        M = cv2.getPerspectiveTransform(input_pts,output_pts)

    cv2.imshow("Image", frame_markers)

    k = cv2.waitKey(5)
    if k == 27:
        break

while True:
    succes, frame = cap.read()

    h,  w = frame.shape[:2]
    newCameraMatrix, roi = cv2.getOptimalNewCameraMatrix(cameraMatrix, dist, (w,h), 1, (w,h))
    dst = cv2.undistort(frame, cameraMatrix, dist, None, newCameraMatrix)
    dst = dst[roi[1]:roi[1]+roi[3], roi[0]:roi[0]+roi[2]]

    # First we make the image monochromatic
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    frame = cv2.warpPerspective(frame,M,(max_width, max_height),flags=cv2.INTER_LINEAR)
    frame = cv2.copyMakeBorder(src=frame, top=30, bottom=15, left=15, right=15, borderType=cv2.BORDER_CONSTANT, value=(255, 255, 255)) 
    cv2.putText(frame, 'Homography', (10, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 1, cv2.LINE_AA, False)
    
    cv2.imshow("frame", frame)
    cv2.imshow("Image", frame_markers)
    
    k = cv2.waitKey(5)
    if k == 27:
        break

# Release and destroy all windows before termination
cap.release()

cv2.destroyAllWindows()