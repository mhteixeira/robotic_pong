import cv2
import numpy as np
import time
from helpers import resize_image, \
					detect_ball, \
					is_ball_inside_field, \
					line_limits, \
					delimit_field, \
					predict_ball_target, \
					y_robot_to_robot_position, \
					non_blocking_move_linear_position

import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from params import *
from pyniryo2 import *

# Open the video or camera
filename = './src/examples/example71.avi'
# cap = cv2.VideoCapture(filename)
cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)

cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1024)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 512)

# Check if camera opened successfully
resize_factor = 100
if (cap.isOpened()== False): 
	print("Error opening video stream or file")
else:
	ret, frame = cap.read()
	frame = resize_image(frame, resize_factor)
	h_frame, w_frame, _ = frame.shape
	size = (w_frame, h_frame)

# result = cv2.VideoWriter(f'./src/examples/processed_{filename.split("/")[-1]}', 
#                          cv2.VideoWriter_fourcc(*'MJPG'),
#                          10, size)
# result = cv2.VideoWriter(f'./src/examples/processed_{int(time.time())}.avi', 
# cv2.VideoWriter_fourcc(*'MJPG'),
# 10, size)
    
# Initializing variables
top_left_corner = np.array([])
bottom_right_corner = np.array([])
x_robot_corner = 0
is_going_to_bounce = False
xd_array = []
yd_array = []
y_pred = h_frame/2
y_preds = []
send_command_to_robot_counter = 0
robot_position = 0.0

# Kalman filter
kf = cv2.KalmanFilter(4, 2)
kf.measurementMatrix = np.array([[1, 0, 0, 0], [0, 1, 0, 0]], np.float32)
kf.transitionMatrix = np.array([[1, 0, 1, 0], [0, 1, 0, 1], [0, 0, 1, 0], [0, 0, 0, 1]], np.float32)


# First make sure that the field is delimited
is_field_delimited = False
while ((not is_field_delimited) and cap.isOpened()):
	ret, frame = cap.read()
	if ret == True:
		frame = resize_image(frame, resize_factor)
		is_field_delimited, output_frame, corners = delimit_field(frame)
		if (is_field_delimited):
			top_left_corner = corners[0]
			bottom_right_corner = corners[1]
			field_height = bottom_right_corner[1] - top_left_corner[1]
			x_robot_corner = bottom_right_corner[0]
			output_frame = cv2.rectangle(output_frame, tuple(top_left_corner), tuple(bottom_right_corner), color=field_limits_rect_color, thickness=2)
		cv2.imshow('Processed',output_frame)
		# Press Q on keyboard to  exit
		if cv2.waitKey(25) & 0xFF == ord('q'):
			break
	else: 
		break

# Initializing and calibrating the robot
robot = NiryoRobot("169.254.200.200")
robot.arm.calibrate_auto()
robot.arm.set_arm_max_velocity(100)
robot.arm.move_linear_pose([0.3, 0.00, 0.15, 0.0, 1.57, 0.0], callback=lambda _: None)

# Start the processing
while(cap.isOpened()):
	ret, frame = cap.read()
	if ret == True:
		frame = resize_image(frame, resize_factor)
		h_frame, w_frame, _ = frame.shape

		# Delimit field again to make the system robust to 
		# changes in position of the arucos and the camera
		is_field_delimited, output_frame, corners = delimit_field(frame)
		if (is_field_delimited):
			top_left_corner = corners[0]
			bottom_right_corner = corners[1]
			field_height = bottom_right_corner[1] - top_left_corner[1]
			x_robot_corner = bottom_right_corner[0]
		output_frame = cv2.rectangle(output_frame, tuple(top_left_corner), tuple(bottom_right_corner), color=field_limits_rect_color, thickness=2)

		# Detect the ball
		is_ball_detected, _, _, x, y, radius = detect_ball(frame, output_frame)
		
		# If the ball is going to bounce, don't predict at the edges, due to instability
		if(is_going_to_bounce):
			ball_inside_field = is_ball_inside_field(x, y, top_left_corner[0], top_left_corner[1] + bounce_margin_size, bottom_right_corner[0], bottom_right_corner[1] - bounce_margin_size)
		else:
			ball_inside_field = is_ball_inside_field(x, y, top_left_corner[0], top_left_corner[1], bottom_right_corner[0], bottom_right_corner[1])
		
		# The prediction only makes sense if the ball is present
		if (is_ball_detected and ball_inside_field):
			output_frame, y_pred, xd_pred, yd_pred, is_going_to_bounce = predict_ball_target(output_frame, kf, [x, y], xd_array, yd_array, x_robot_corner, top_left_corner, bottom_right_corner, y_preds, is_going_to_bounce, y_pred)
			output_frame = cv2.circle(output_frame, (int(x), int(y)), radius=int(radius), color=(100, 255, 100), thickness=2)
			output_frame = cv2.arrowedLine(output_frame, (int(x), int(y)), (int(x+10*xd_pred), int(y+10*yd_pred)), (255, 0, 0), 2) 
		else:
			xd_array = []
			yd_array = []
		
		y_preds.append(y_pred)
		y_robot = y_pred if len(y_preds) > 20 else np.mean(y_preds[-20:])
		if (is_going_to_bounce):
			output_frame = cv2.line(output_frame, (top_left_corner[0], top_left_corner[1] + bounce_margin_size),  (bottom_right_corner[0], top_left_corner[1] + bounce_margin_size), color=(0, 0, 255), thickness=1)
			output_frame = cv2.line(output_frame, (top_left_corner[0], bottom_right_corner[1] - bounce_margin_size), (bottom_right_corner[0], bottom_right_corner[1] - bounce_margin_size), color=(0, 0, 255), thickness=1)
		output_frame = cv2.rectangle(output_frame, (int(x_robot_corner - 6), int(y_robot - 30)), (int(x_robot_corner + 6), int(y_robot + 30)), color=(255, 255, 255), thickness=-1)
		
		new_robot_position = y_robot_to_robot_position(y_robot, top_left_corner, bottom_right_corner)
		if abs(robot_position - new_robot_position) > 0.07:
			robot.arm.stop_move()
			time.sleep(0.001)
			robot.arm.move_joints([new_robot_position, -1.552, 1.29, 0.0, -1.2, 0.0])
			# robot.arm.move_pose([0.3, new_robot_position, 0.15, 0.0, 1.57, 0.0])
			# non_blocking_move_linear_position(robot, [0.3, new_robot_position, 0.15, 0.0, 1.57, 0.0])
			send_command_to_robot_counter = 1
		elif abs(robot_position - new_robot_position) > 0.02:
			robot.arm.move_joints([new_robot_position, -1.552, 1.29, 0.0, -1.2, 0.0])
			# robot.arm.move_pose([0.3, new_robot_position, 0.15, 0.0, 1.57, 0.0])
			# non_blocking_move_linear_position(robot, [0.3, new_robot_position, 0.15, 0.0, 1.57, 0.0])
		robot_position = new_robot_position
		
		cv2.imshow('Processed', output_frame)
		# result.write(output_frame)
		
		# Press Q on keyboard to  exit
		if cv2.waitKey(25) & 0xFF == ord('q'):
			break

	# Break the loop
	else: 
		break
# Closing the connection with the robot
robot.end()

# When everything done, release the video capture and video write objects
cap.release()
# result.release()

# Closes all the frames
cv2.destroyAllWindows()