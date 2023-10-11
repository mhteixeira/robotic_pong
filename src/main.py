import cv2
import numpy as np

import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, Button

from pyniryo2 import *

import pickle
import serial
import time
import sys

from params import *
from helpers import detect_ball, is_ball_inside_field, non_blocking_move_linear_position, warp_point, predict_target, custom_set_arm_max_velocity
from SerialStepperMotor import SerialStepperMotor

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

################################
# CONFIGURING INTERACTIVE PLOT #
################################

# General plot configurations
fig, ax = plt.subplots(figsize=[6, 4.2])

# Close the program on pressing 'q'
fig_is_active = True

def on_key_press(event):
	global fig_is_active
	sys.stdout.flush()
	if event.key == 'q':
		fig_is_active = False
	

fig.canvas.mpl_connect('key_press_event', on_key_press)

# Make a horizontal slider to control the angle to hit the ball
slider_ax = fig.add_axes([0.28, 0.035, 0.2, 0.03])
slider = Slider(
	ax=slider_ax,
	label='Angle (°)  ',
	valmin = -45,
	valmax = 45,
	valinit = 0,
)

# Create a Button to throw the ball
button_ax = fig.add_axes([0.705, 0.035, 0.15, 0.05])
button = Button(button_ax, 'Throw ball!', hovercolor='0.975')

throwing_movement_in_course = False

def update_throwing_flag(_):
	print('Movement completed')
	throwing_movement_in_course = False
	non_blocking_move_linear_position(robot, initial_pose)

def throw_ball(event):
	global final_pose
	throwing_movement_in_course = True
	print("Throwing the ball")
	robot.arm.stop_move()
	time.sleep(0.001)
	non_blocking_move_linear_position(robot, final_pose, callback=update_throwing_flag)

button.on_clicked(throw_ball)

plt.suptitle("PIP-PRoS")
ax.set_title("Initializing the program", color='gray', fontsize=10)
output_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
button_ax.set_visible(False)
slider_ax.set_visible(False)
image_ax = ax.imshow(output_frame)
plt.draw()
plt.pause(0.01)

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

ax.set_title("Identifying the ArUcos", color='gray', fontsize=10)

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
	
	output_frame = cv2.cvtColor(frame_markers, cv2.COLOR_BGR2RGB)
	image_ax.set_data(output_frame)

	if fig_is_active:
		plt.draw()
		plt.pause(0.01)
	else:
		plt.ioff()
		break
	
plt.pause(0.5)

if not field_delimited:
	sys.exit()


################################
# INITIALIZING SERIAL TO MOTOR #
################################
print(serial_port_motor)
motor = SerialStepperMotor(serial_port_motor, baudrate_motor, pos_min_motor, pos_max_motor)

if not motor.is_connected():
	sys.exit()

motor.move_to(0.5)

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

# Variables to deal with the ball throwing
ready_to_throw = False
if throwing_mode == 1:
	ax.set_title("Manual ball control", color='black', fontsize=10)
if throwing_mode == 2:
	ax.set_title("Put the ball inside the delimited region", color='red', fontsize=10)

button_ax.set_visible(False)
slider_ax.set_visible(False)
previous_movement = None

while True:
	succes, frame = cap.read()

	dst = cv2.undistort(frame, cameraMatrix, dist, None, newCameraMatrix)
	frame = dst[roi[1]:roi[1]+roi[3], roi[0]:roi[0]+roi[2]]
	output_frame = frame.copy()
	
	# Entering homography frame 
	h, w, _ = frame.shape
	homography = cv2.warpPerspective(frame,M,(max_width, max_height),flags=cv2.INTER_LINEAR)
	homography = cv2.line(homography, (hit_region, 0), (hit_region, max_height), color=(0, 255, 255), thickness=1)
	
	# Detect the ball
	is_ball_detected, _, _, x, y, radius = detect_ball(homography, homography)
	
	# Check if the ball is inside the field (close to the wall)
	if(is_going_to_bounce):
		ball_inside_field = is_ball_inside_field(x, y, hit_region, bounce_margin_size, max_width, max_height - bounce_margin_size)
	else:
		ball_inside_field = is_ball_inside_field(x, y, hit_region, 0, max_width, max_height)
	
	# The prediction only makes sense if the ball is present
	if (is_ball_detected and ball_inside_field):
		homography, y_pred, xd_pred, yd_pred, is_going_to_bounce = predict_target(homography, kf, [x, y], xd_array, yd_array, x_robot_corner, max_width, max_height, y_preds, is_going_to_bounce, y_pred)
		homography = cv2.arrowedLine(homography, (int(x), int(y)), (int(x+10*xd_pred), int(y+10*yd_pred)), (255, 0, 0), 2) 
	else:
		xd_array = []
		yd_array = []

	y_preds.append(y_pred)
	y_robot = y_pred if len(y_preds) > 20 else np.mean(y_preds[-20:])
	
	if not previous_y_robot:
		previous_y_robot = y_robot
	else:
		if abs(y_robot - previous_y_robot) > 50:
			motor.stop()
			position_percentage = y_robot/max_height
			motor.move_to(position_percentage)
			previous_y_robot = y_robot
			
	if (is_going_to_bounce):
		homography = cv2.line(homography, (0, bounce_margin_size),  (max_width, bounce_margin_size), color=(0, 0, 255), thickness=1)
		homography = cv2.line(homography, (0, max_height - bounce_margin_size), (max_width, max_height - bounce_margin_size), color=(0, 0, 255), thickness=1)
	
	if throwing_mode == 2:
		# Check if the ball is inside the hitting region
		ball_inside_hit_region = is_ball_inside_field(x, y, 0, 0, hit_region, max_height) if is_ball_detected else False
	
		if (ready_to_throw == False) and (ball_inside_hit_region == True):
			ax.set_title("Ready to throw!", color='green', fontsize=10)
			button_ax.set_visible(True)
			slider_ax.set_visible(True)
			ready_to_throw = True
		elif (ready_to_throw == True) and (ball_inside_hit_region == False):
			ax.set_title("Put the ball inside the delimited region", color='red', fontsize=10)
			button_ax.set_visible(False)
			slider_ax.set_visible(False)
			ready_to_throw = False

		# Calculating arm trajectory to throw the ball
		slope = np.tan(-np.deg2rad(slider.val))
		initial_position = slope*(-int(x)) + int(y)
		final_position = slope*(hit_region - int(x)) + int(y)

		initial_pose = aruco_0_pose + (aruco_1_pose - aruco_0_pose)*initial_position/max_height
		final_pose = aruco_0_pose + (aruco_1_pose - aruco_0_pose)*final_position/max_height
		final_pose[0] = 0.4

		if ball_inside_hit_region and not throwing_movement_in_course:
			if (initial_pose[1] > aruco_0_pose[1]) or (initial_pose[1] < aruco_1_pose[1]):
				print("Invalid initial position: out of field")
			initial_pose[1] = min(aruco_0_pose[1], initial_pose[1])
			initial_pose[1] = max(aruco_1_pose[1], initial_pose[1])
			
			if (final_pose[1] > aruco_0_pose[1]) or (final_pose[1] < aruco_1_pose[1]):
				print("Invalid final position: out of field")
			final_pose[1] = min(aruco_0_pose[1], final_pose[1])
			final_pose[1] = max(aruco_1_pose[1], final_pose[1])

			if previous_movement:
				movement = previous_movement - initial_position
				# if abs(movement) > 5:
				robot.arm.stop_move()
				time.sleep(0.001)
				non_blocking_move_linear_position(robot, initial_pose)
				previous_movement = initial_position
			else:
				robot.arm.stop_move()
				time.sleep(0.001)
				non_blocking_move_linear_position(robot, initial_pose)
				previous_movement = initial_position

	# Returning to the original frame
	inversed_homography = cv2.warpPerspective(homography,IM,(w, h))
	output_frame[inversed_homography != 0] = 0
	output_frame = output_frame + inversed_homography
	output_frame = cv2.line(output_frame, top_left_corner, bottom_left_corner, color=(100, 0, 0), thickness=2)
	output_frame = cv2.line(output_frame, top_left_corner, top_right_corner, color=(100, 0, 0), thickness=2)
	output_frame = cv2.line(output_frame, bottom_right_corner, top_right_corner, color=(100, 0, 0), thickness=2)
	output_frame = cv2.line(output_frame, bottom_right_corner, bottom_left_corner, color=(100, 0, 0), thickness=2)
	
	if throwing_mode == 2:
		# Direction of the throw
		if ball_inside_hit_region:
			x, y = warp_point(int(x), int(y), IM)
			output_frame = cv2.arrowedLine(output_frame, (int(x), int(y)), (int(x+40*np.cos(-np.deg2rad(slider.val))), int(y+40*np.sin(-np.deg2rad(slider.val)))), (0, 0, 0), 2) 
			initial_pos_x, initial_pos_y = warp_point(0, int(initial_position), IM)
			output_frame = cv2.circle(output_frame, (initial_pos_x, initial_pos_y), radius=2, color=(0, 0, 255), thickness=2)
			final_pos_x, final_pos_y = warp_point(hit_region, int(final_position), IM)
			output_frame = cv2.circle(output_frame, (final_pos_x, final_pos_y), radius=2, color=(0, 0, 255), thickness=2)
	
	x_robot, y_robot = warp_point(int(x_robot_corner), int(y_robot), IM)
	output_frame = cv2.rectangle(output_frame, (int(x_robot - 6), int(y_robot - 30)), (int(x_robot + 6), int(y_robot + 30)), color=(255, 255, 255), thickness=-1)
	
	output_frame = cv2.cvtColor(output_frame, cv2.COLOR_BGR2RGB)
	image_ax.set_data(output_frame)

	if fig_is_active:
		plt.draw()
		plt.pause(0.01)
	else:
		plt.ioff()
		break

#######################
# CLOSING CONNECTIONS #
#######################

# Closing the connection with the robot
if throwing_mode == 2:
	robot.end()

motor.move_to(0)
time.sleep(1)
motor.close()