import cv2
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.widgets import Slider, Button
from helpers import detect_ball, is_ball_inside_field, non_blocking_move_linear_position
import sys
import pickle
from params import *
import time
from pyniryo2 import *

######################
# INITIALIZING NIRYO #
######################

robot = NiryoRobot("169.254.200.200")
robot.arm.calibrate_auto()
robot.arm.set_arm_max_velocity(100)
non_blocking_move_linear_position(robot, (aruco_0_pose + aruco_1_pose)/2)
final_pose = aruco_0_pose

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
	ret, frame = cap.read()
	ret, frame = cap.read()
	ret, frame = cap.read()
	h, w, _ = frame.shape

newCameraMatrix, roi = cv2.getOptimalNewCameraMatrix(cameraMatrix, dist, (w,h), 1, (w,h))
dst = cv2.undistort(frame, cameraMatrix, dist, None, newCameraMatrix)
frame = dst[roi[1]:roi[1]+roi[3], roi[0]:roi[0]+roi[2]]

################################
# CONFIGURING INTERACTIVE PLOT #
################################

plt.ion()
fig, ax = plt.subplots(figsize=[6, 4.2])
fig_is_active = True

def on_press(event):
	global fig_is_active
	sys.stdout.flush()
	if event.key == 'q':
		fig_is_active = False

fig.canvas.mpl_connect('key_press_event', on_press)

# Make a horizontal slider to control the frequency.
slider_ax = fig.add_axes([0.28, 0.035, 0.2, 0.03])
slider = Slider(
	ax=slider_ax,
	label='Angle (°)  ',
	valmin = -45,
	valmax = 45,
	valinit = 0,
)

# The function to be calledq anytime a slider's value changes
# def update(val):
# 	print(slider.val)

# register the update function with each slider
# slider.on_changed(update)

# Create a `matplotlib.widgets.Button` to reset the sliders to initial values.
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
ax.set_xticks([])
ax.set_yticks([])

output_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
image_ax = ax.imshow(output_frame)

plt.draw()
plt.pause(0.01)

######################q
# IDENTIFYING ARUCOS #
######################

field_delimited = False

# Transformation matrix for the homography
M = None
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

	dst = cv2.undistort(frame, cameraMatrix, dist, None, newCameraMatrix)
	frame = dst[roi[1]:roi[1]+roi[3], roi[0]:roi[0]+roi[2]]
	output_frame = dst.copy()
	
	# First we make the image monochromatic
	gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

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

		pixel_density = max_height/field_lenght_m

		# Compute the perspective transform M
		M = cv2.getPerspectiveTransform(input_pts,output_pts)

	output_frame = cv2.cvtColor(frame_markers, cv2.COLOR_BGR2RGB)
	image_ax.set_data(output_frame)

	if fig_is_active:
		plt.draw()
		plt.pause(0.01)
	else:
		plt.ioff()
		break
	
plt.pause(0.5)
_, IM = cv2.invert(M)

#############
# MAIN LOOP #
##############

def warp_point(x: int, y: int, M):
	d = M[2, 0] * x + M[2, 1] * y + M[2, 2]

	return (
		int((M[0, 0] * x + M[0, 1] * y + M[0, 2]) / d), # x
		int((M[1, 0] * x + M[1, 1] * y + M[1, 2]) / d), # y
	)


previous_movement = None

ready_to_throw = False
ax.set_title("Put the ball inside the delimited region", color='red', fontsize=10)
button_ax.set_visible(False)
slider_ax.set_visible(False)

while True:

	succes, frame = cap.read()

	dst = cv2.undistort(frame, cameraMatrix, dist, None, newCameraMatrix)
	frame = dst[roi[1]:roi[1]+roi[3], roi[0]:roi[0]+roi[2]]
	output_frame = frame.copy()
	
	# Entering homography frame 
	h, w, _ = frame.shape
	homography = cv2.warpPerspective(frame,M,(max_width, max_height),flags=cv2.INTER_LINEAR)
	homography = cv2.line(homography, (hit_region, 0), (hit_region, max_height), color=(0, 255, 255), thickness=1)
	
	is_ball_detected, _, _, x, y, radius = detect_ball(homography, homography)
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
		if previous_movement:
			movement = previous_movement - initial_position
			if abs(movement) > 5:
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
	

	# Direction of the throw
	if ball_inside_hit_region:
		x, y = warp_point(int(x), int(y), IM)
		output_frame = cv2.arrowedLine(output_frame, (int(x), int(y)), (int(x+40*np.cos(-np.deg2rad(slider.val))), int(y+40*np.sin(-np.deg2rad(slider.val)))), (0, 0, 0), 2) 
		initial_pos_x, initial_pos_y = warp_point(0, int(initial_position), IM)
		output_frame = cv2.circle(output_frame, (initial_pos_x, initial_pos_y), radius=2, color=(0, 0, 255), thickness=2)
		final_pos_x, final_pos_y = warp_point(hit_region, int(final_position), IM)
		output_frame = cv2.circle(output_frame, (final_pos_x, final_pos_y), radius=2, color=(0, 0, 255), thickness=2)
		
	
	output_frame = cv2.cvtColor(output_frame, cv2.COLOR_BGR2RGB)
	image_ax.set_data(output_frame)

	if fig_is_active:
		plt.draw()
		plt.pause(0.01)
	else:
		plt.ioff()
		break

# Closing the connection with the robot
robot.end()