import cv2
import numpy as np

import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, Button

from pyniryo2 import *

import threading
import pickle
import serial
import time
import sys

from params import *
from helpers import detect_ball, is_ball_inside_field, non_blocking_move_linear_position, warp_point, predict_target, custom_set_arm_max_velocity
from SerialStepperMotor import SerialStepperMotor

#####################################
# INTERNAL PARAMETERS FOR DEBUGGING #
#####################################

output_processing_times = True
processing_times = []

##############################
# OPEN CV CAMERA CONFIGURING #
##############################

h, w = 640, 480
cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
time.sleep(2)

# Load camera calibration parameters
file = open("./assets/calibration/calibration.pkl",'rb')
cameraMatrix, dist = pickle.load(file)

frame = None
if (cap.isOpened()== False): 
    print("Error opening video stream or file")
else:
    for _ in range(2):
        ret, frame = cap.read()
    h, w, _ = frame.shape

# Undistorting the frame
newCameraMatrix, roi = cv2.getOptimalNewCameraMatrix(cameraMatrix, dist, (w,h), 1, (w,h))
dst = cv2.undistort(frame, cameraMatrix, dist, None, newCameraMatrix)
frame = dst[roi[1]:roi[1]+roi[3], roi[0]:roi[0]+roi[2]]

############################################
# CONFIGURING MENU TO CHOOSE THROWING MODE #
############################################

# Set the plot as interactive
plt.ion()

throwing_mode = None # 1 for Manual; 2 for Niryo

def set_manual_mode(event):
    global throwing_mode
    throwing_mode = 1

def set_arm_mode(event):
    global throwing_mode
    throwing_mode = 2

fig, ax = plt.subplots(figsize=[4, 1.4])
ax.set_title("Choose an operation mode")
ax.axis('off')
manual_button_ax = fig.add_axes([0.1, 0.2, 0.35, 0.4])
manual_button = Button(manual_button_ax, 'Manual', hovercolor='0.975')
manual_button.on_clicked(set_manual_mode)
arm_button_ax = fig.add_axes([0.5, 0.2, 0.35, 0.4])
arm_button = Button(arm_button_ax, 'Niryo', hovercolor='0.975')
arm_button.on_clicked(set_arm_mode)
plt.tight_layout()

while throwing_mode == None:
    if not plt.fignum_exists(1):
        fig, ax = plt.subplots(figsize=[4, 1.4])
        ax.set_title("Choose an operation mode")
        ax.axis('off')
        manual_button_ax = fig.add_axes([0.1, 0.2, 0.35, 0.4])
        manual_button = Button(manual_button_ax, 'Manual', hovercolor='0.975')
        manual_button.on_clicked(set_manual_mode)
        arm_button_ax = fig.add_axes([0.5, 0.2, 0.35, 0.4])
        arm_button = Button(arm_button_ax, 'Niryo', hovercolor='0.975')
        arm_button.on_clicked(set_arm_mode)
        plt.tight_layout()
    plt.draw()
    plt.pause(0.1)

plt.close(fig)

######################
# IDENTIFYING ARUCOS #
######################

field_delimited = False

# Transformation matrix for the homography
M = None
IM = None # The inverse of M
max_width = None
max_height = None

# Relevant corners for the homography
corners, ids = None, None
top_left_corner = None
bottom_right_corner = None
top_right_corner = None
bottom_left_corner = None

# Configuring the ArUco detector
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
parameters =  cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)

while not field_delimited:
    succes, frame = cap.read()

    # Undistorting the frame from the camera
    dst = cv2.undistort(frame, cameraMatrix, dist, None, newCameraMatrix)
    frame = dst[roi[1]:roi[1]+roi[3], roi[0]:roi[0]+roi[2]]
    output_frame = dst.copy()

    # First we make the image monochromatic
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Then we detect the arucos
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
        top_right_corner_id = np.where(ids == 4)[0]

        # The bottom right corner is identified by the Aruco with ID = 1
        bottom_left_corner_id = np.where(ids == 1)[0]

        top_left_corner = corners[top_left_corner_id][0][0][0]
        bottom_right_corner = corners[bottom_right_corner_id][0][0][0]
        top_right_corner = corners[top_right_corner_id][0][0][0]
        bottom_left_corner = corners[bottom_left_corner_id][0][0][0]

        width_top = np.hypot(
            top_left_corner[0] - top_right_corner[0], 
            top_left_corner[1] - top_right_corner[1])
        width_bottom = np.hypot(
            bottom_left_corner[0] - bottom_right_corner[0], 
            bottom_left_corner[1] - bottom_right_corner[1])
        max_width = max(int(width_top), int(width_bottom))
        
        height_left = np.hypot(
            top_left_corner[0] - bottom_left_corner[0], 
            top_left_corner[1] - bottom_left_corner[1])
        height_right = np.hypot(
            top_right_corner[0] - bottom_right_corner[0], 
            top_right_corner[1] - bottom_right_corner[1])
        max_height = max(int(height_left), int(height_right))

        input_pts = np.float32([top_left_corner, 
                                bottom_left_corner, 
                                bottom_right_corner, 
                                top_right_corner])
        output_pts = np.float32([[0, 0],
                                [0, max_height - 1],
                                [max_width - 1, max_height - 1],
                                [max_width - 1, 0]])

        pixel_density = max_height/field_lenght_m
        
        # Compute the perspective transform M
        M = cv2.getPerspectiveTransform(input_pts,output_pts)
        _, IM = cv2.invert(M)
    
    cv2.imshow('Frame', output_frame)

    # Break the loop if 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

if not field_delimited:
    print("Field not delimited")
    sys.exit()

cv2.destroyAllWindows()

################################
# INITIALIZING SERIAL TO MOTOR #
################################
print(serial_port_motor)
motor = SerialStepperMotor(serial_port_motor, baudrate_motor)

if not motor.is_connected():
    print("Motor not connected")
    sys.exit()

motor.calibrate()

######################
# INITIALIZING NIRYO #
######################

if throwing_mode == 2:
    robot = NiryoRobot("169.254.200.200")
    robot.arm.calibrate_auto()
    custom_set_arm_max_velocity(robot.arm, 200)
    non_blocking_move_linear_position(robot, (aruco_0_pose + aruco_1_pose)/2)
    final_pose = aruco_0_pose

#############
# MAIN LOOP #
#############

# Variables to deal with the movement prediction
is_going_to_bounce = False
xd_array = []
yd_array = []
y_preds = []
y_pred = max_height/2
x_robot_corner = max_width
previous_y_robot = None

# Kalman filter
kf = cv2.KalmanFilter(4, 2)
kf.measurementMatrix = np.array([[1, 0, 0, 0], [0, 1, 0, 0]], np.float32)
kf.transitionMatrix = np.array([[1, 0, 1, 0], [0, 1, 0, 1], [0, 0, 1, 0], [0, 0, 0, 1]], np.float32)

previous_movement = None
previous_frame = None

while True:
    t_0 = time.time()
    
    succes, frame = cap.read()
    if (previous_frame == frame).all():
        continue
    t_frame = time.time()
    
    dst = cv2.undistort(frame, cameraMatrix, dist, None, newCameraMatrix)
    frame = dst[roi[1]:roi[1]+roi[3], roi[0]:roi[0]+roi[2]]
    output_frame = frame.copy()
    
    # Entering homography frame 
    h, w, _ = frame.shape
    homography = cv2.warpPerspective(frame,M,(max_width, max_height),flags=cv2.INTER_LINEAR)
    homography = cv2.line(homography, (hit_region, 0), (hit_region, max_height), color=(0, 255, 255), thickness=1)
    
    t_distortions = time.time()

    # Detect the ball
    is_ball_detected, _, _, x, y, radius = detect_ball(homography, homography)
    
    t_ball_detection = time.time()

    # Check if the ball is inside the field (close to the wall)
    if(is_going_to_bounce):
        ball_inside_field = is_ball_inside_field(x, y, 0, bounce_margin_size, max_width, max_height - bounce_margin_size)
    else:
        ball_inside_field = is_ball_inside_field(x, y, 0, 0, max_width, max_height)
    
    # The prediction only makes sense if the ball is present
    if (is_ball_detected and ball_inside_field):
        homography, y_pred, xd_pred, yd_pred, is_going_to_bounce = predict_target(homography, kf, [x, y], xd_array, yd_array, x_robot_corner, max_width, max_height, y_preds, is_going_to_bounce, y_pred)
        homography = cv2.arrowedLine(homography, (int(x), int(y)), (int(x+10*xd_pred), int(y+10*yd_pred)), (255, 0, 0), 2) 
    else:
        xd_array = []
        yd_array = []


    y_preds.append(y_pred)
    y_robot = y_pred if len(y_preds) > 20 else np.mean(y_preds[-20:])
    
    t_prediction = time.time()
    # if xd_array == []:
    # 	y_robot = max_height/2
    if not previous_y_robot:
        previous_y_robot = y_robot
    else:
        if abs(y_robot - previous_y_robot) > 20:
            # motor.stop()
            position_percentage = y_robot/max_height
            motor.move_to(position_percentage)
            previous_y_robot = y_robot
            
    t_moving_motor = time.time()

    if (is_going_to_bounce):
        homography = cv2.line(homography, (0, bounce_margin_size),  (max_width, bounce_margin_size), color=(0, 0, 255), thickness=1)
        homography = cv2.line(homography, (0, max_height - bounce_margin_size), (max_width, max_height - bounce_margin_size), color=(0, 0, 255), thickness=1)
    
    # Returning to the original frame
    inversed_homography = cv2.warpPerspective(homography,IM,(w, h))
    output_frame[inversed_homography != 0] = 0
    output_frame = output_frame + inversed_homography
    output_frame = cv2.line(output_frame, top_left_corner, bottom_left_corner, color=(100, 0, 0), thickness=2)
    output_frame = cv2.line(output_frame, top_left_corner, top_right_corner, color=(100, 0, 0), thickness=2)
    output_frame = cv2.line(output_frame, bottom_right_corner, top_right_corner, color=(100, 0, 0), thickness=2)
    output_frame = cv2.line(output_frame, bottom_right_corner, bottom_left_corner, color=(100, 0, 0), thickness=2)
    
    x_robot, y_robot = warp_point(int(x_robot_corner), int(y_robot), IM)
    output_frame = cv2.rectangle(output_frame, (int(x_robot - 6), int(y_robot - 30)), (int(x_robot + 6), int(y_robot + 30)), color=(255, 255, 255), thickness=-1)
    
    cv2.imshow('Frame', output_frame)

    # Break the loop if 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

    t_plot = time.time()
    processing_times.append([
        is_ball_detected,
        t_0,
        t_frame,
        t_distortions,
        t_ball_detection,
        t_prediction,
        t_moving_motor,
        t_plot
    ])

#######################
# CLOSING CONNECTIONS #
#######################

filename = 'main_opencv_plot_13_11.txt'
with open(filename, 'w') as f:
    for line in processing_times:
        print(*line, sep=", ", file=f)


cv2.destroyAllWindows()
motor.move_to(0.5)
time.sleep(1)
motor.close()