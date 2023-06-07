import cv2
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.widgets import Slider, Button
from helpers import detect_ball
import sys
import pickle
from params import *
import time

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
print(frame)

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
def update(val):
	print(slider.val)
# register the update function with each slider
slider.on_changed(update)

# Create a `matplotlib.widgets.Button` to reset the sliders to initial values.
button_ax = fig.add_axes([0.705, 0.035, 0.15, 0.05])
button = Button(button_ax, 'Throw ball!', hovercolor='0.975')

def throw_ball(event):
	print(slider.val)

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
	
plt.pause(2)
print(M)

#############
# MAIN LOOP #
##############

ax.set_title("Ready to play!", color='green', fontsize=10)

while True:

	succes, frame = cap.read()

	dst = cv2.undistort(frame, cameraMatrix, dist, None, newCameraMatrix)
	frame = dst[roi[1]:roi[1]+roi[3], roi[0]:roi[0]+roi[2]]
	output_frame = frame.copy()
	

	# output_frame = cv2.line(output_frame)
	
	is_ball_detected, _, _, x, y, radius = detect_ball(frame, output_frame)
	
	
	# Direction of the throw
	output_frame = cv2.arrowedLine(output_frame, (int(x), int(y)), (int(x+40*np.cos(-np.deg2rad(slider.val))), int(y+40*np.sin(-np.deg2rad(slider.val)))), (255, 0, 0), 2) 
	
	
	output_frame = cv2.cvtColor(output_frame, cv2.COLOR_BGR2RGB)
	image_ax.set_data(output_frame)

	if fig_is_active:
		plt.draw()
		plt.pause(0.01)
	else:
		plt.ioff()
		break