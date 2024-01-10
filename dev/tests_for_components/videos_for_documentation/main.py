import cv2
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.widgets import Slider, Button
from helpers import detect_ball, is_ball_inside_field, non_blocking_move_linear_position, warp_point, predict_target, custom_set_arm_max_velocity
import sys
import pickle
from params import *
import time
from pyniryo2 import *


# Set the plot as interactive
plt.ion()

fig, axes = plt.subplots(ncols=3, figsize=[13, 3])

# Close the program on pressing 'q'
fig_is_active = True

def on_press(event):
	global fig_is_active
	sys.stdout.flush()
	if event.key == 'q':
		fig_is_active = False

fig.canvas.mpl_connect('key_press_event', on_press)
# plt.suptitle("Identificação do campo")
axes[0].set_title("Detecção dos arucos", fontsize=10)
axes[1].set_title("Isolar o campo para processamento", fontsize=10)
axes[2].set_title("Visualização final", fontsize=10)
for ax in axes:
	ax.set_xticks([])
	ax.set_yticks([])
image_axes = None

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

while True:
	succes, frame = cap.read()
	if not image_axes:
		image_axes = [
			axes[0].imshow(frame),
			axes[1].imshow(frame),
			axes[2].imshow(frame),
		]
	homography = frame.copy()
	output_image = frame.copy()

	# Undistorting the frame from the camera
	dst = cv2.undistort(frame, cameraMatrix, dist, None, newCameraMatrix)
	frame = dst[roi[1]:roi[1]+roi[3], roi[0]:roi[0]+roi[2]]
	output_frame = dst.copy()
	
	# First we make the image monochromatic
	gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

	# Then we detect the arucos
	corners, ids, rejectedImgPoints = detector.detectMarkers(gray)
	frame_markers = cv2.aruco.drawDetectedMarkers(frame.copy(), corners, ids)
	# frame_markers = frame.copy()
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

		# The bottom right corner is identified by the Aruco with ID = 1
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
		_, IM = cv2.invert(M)
		homography = cv2.warpPerspective(frame,M,(max_width, max_height),flags=cv2.INTER_LINEAR)
		homography = cv2.line(homography, (hit_region, 0), (hit_region, max_height), color=(0, 255, 255), thickness=1)
		homography2 = cv2.cvtColor(homography, cv2.COLOR_BGR2RGB)
		image_axes[1].set_data(homography2)
		# cv2.imshow('Homography', homography)

		inversed_homography = cv2.warpPerspective(homography,IM,(w, h))
		output_frame2 = output_frame.copy()
		output_frame2[inversed_homography != 0] = 0
		output_frame2 = output_frame2 + inversed_homography
		output_frame2 = cv2.line(output_frame2, top_left_corner, bottom_left_corner, color=(100, 0, 0), thickness=2)
		output_frame2 = cv2.line(output_frame2, top_left_corner, top_right_corner, color=(100, 0, 0), thickness=2)
		output_frame2 = cv2.line(output_frame2, bottom_right_corner, top_right_corner, color=(100, 0, 0), thickness=2)
		output_frame2 = cv2.line(output_frame2, bottom_right_corner, bottom_left_corner, color=(100, 0, 0), thickness=2)
		output_frame2 = cv2.cvtColor(output_frame2, cv2.COLOR_BGR2RGB)
		image_axes[2].set_data(output_frame2)
		# cv2.imshow('Reconstructed', output_frame2)
		

	output_frame = cv2.cvtColor(frame_markers, cv2.COLOR_BGR2RGB)
	# output_frame = frame_markers
	image_axes[0].set_data(output_frame)
	# cv2.imshow('Processed', output_frame)
	
	# k = cv2.waitKey(5)
	# if k == 27:
	# 	break
	# elif k == ord('s'): # wait for 's' key to save and exit
	# 	cv2.imwrite('assets/examples/delimited_field.png', output_frame)
	# 	cv2.imwrite('assets/examples/homography.png', homography)
	# 	cv2.imwrite('assets/examples/reconstructed.png', output_frame2)
	# 	print("image saved!")

	
	if fig_is_active:
		plt.draw()
		plt.pause(0.01)
	else:
		plt.ioff()
		break

plt.pause(0.5)
_, IM = cv2.invert(M)