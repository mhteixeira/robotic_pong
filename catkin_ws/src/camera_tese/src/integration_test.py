#!/usr/bin/env python


# python ball_tracking.py --video ball_tracking_example.mp4


# import the necessary packages
from collections import deque
from cv2 import norm
from six.moves import urllib
from imutils.video import VideoStream
from kalmanfilter import KalmanFilter
from calc_speed import Calc_Speed
from scipy.spatial import distance as dist
from imutils import perspective
from imutils import contours
from pyzbar import pyzbar
from pyzbar.pyzbar import decode
from niryo_one_tcp_client import *
from itertools import product, combinations
import time
import random
import numpy as np
import math as m
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
import cv2.aruco as aruco
import matplotlib as mpl
import rospy
import sys
import argparse
import cv2
import imutils
import time
import datetime
import math

print(cv2.__version__)

#functions

# compute angle 1
def compute_t1(TL_pos_world):
  res = m.atan2(TL_pos_world[1], TL_pos_world[0])

  if (TL_pos_world[0] > 0 or TL_pos_world[0] == 0):
    return res
  else:
    if (TL_pos_world[1] < 0):
      return res - m.pi
    else:
      return res + m.pi

def compute_HL_pos(L7, TL_pos_shoulder):
  return np.array([TL_pos_shoulder[0], TL_pos_shoulder[1], TL_pos_shoulder[2] + L7]).T

def compute_WL_pos(L6x, L6z, HL_pos_shoulder):
  return np.array([HL_pos_shoulder[0] + L6z, HL_pos_shoulder[1], HL_pos_shoulder[2] + L6x]).T

def compute_t2_to_t6(L2, L3, L4x, L4z, L5, WL_pos_shoulder, t1):
  x, z = WL_pos_shoulder[0], WL_pos_shoulder[2]
  
  # length of auxiliary arm between elbow link and wrist link 
  L45 = m.sqrt(L4z**2 + (L5 + L4x)**2)

  # alpha angle
  alpha = m.atan2(L4z, L5 + L4x)

  # beta angle
  beta = m.acos((x**2+(z-L2)**2-L3**2-L45**2)/(2*L3*L45))

  # gama angle
  gama = m.atan2(z-L2, x)

  # phi angle
  phi = m.acos((x**2+(z-L2)**2+L3**2-L45**2)/(2*m.sqrt(x**2+(z-L2)**2)*L3))

  # compute t2 to t6
  t2 = - m.pi/2 + gama + phi
  t3 = m.pi/2 - beta - alpha
  t4 = t1
  t5 = - t2 - t3 - alpha + 0.15 # ??? 0.15 to reach correct angle
  t6 = -t1/2
  
  return t2, t3, t4, t5, t6

def world_to_shoulder(t1, L1, pos_world):
  # transformation from base link to shoulder link
  T_BS = np.array([[m.cos(t1), m.sin(t1), 0, 0],
                   [-m.sin(t1), m.cos(t1), 0, 0],
                   [0, 0, 1, -L1],
                   [0, 0, 0, 1]])

  # apply transformation to position
  pos_shoulder = np.matmul(T_BS, cartesian_to_homogeneous(pos_world))

  return homogeneous_to_cartesian(pos_shoulder)

# transforms vector from homogeneous to cartesian coordinates
def homogeneous_to_cartesian(hmg):
  return np.delete(hmg, 3)

# transforms vector from cartesian to homogeneous coordinates
def cartesian_to_homogeneous(cart):
  return np.append(cart.T, 1).T

# compute tool link position in world referential based on joint angles
def direct_kinematics(angles, L):
  [t1, t2, t3, t4, t5, t6] = angles
  [L1, L2, L3, L4x, L4z, L5, L6x, L6z, L7] = L

  # transformation from shoulder link to base link
  T_SB = np.array([[m.cos(t1), -m.sin(t1), 0, 0],
                   [m.sin(t1), m.cos(t1), 0, 0],
                   [0, 0, 1, L1],
                   [0, 0, 0, 1]])
  
  # transformation from arm link to shoulder link
  T_AS = np.array([[-m.sin(t2), -m.cos(t2), 0, 0],
                   [0, 0, -1, 0],
                   [m.cos(t2), -m.sin(t2), 0, L2],
                   [0, 0, 0, 1]])

  # transformation from elbow link to arm link
  T_EA = np.array([[m.sin(t3), m.cos(t3), 0, L3],
                   [-m.cos(t3), m.sin(t3), 0, 0],
                   [0, 0, 1, 0],
                   [0, 0, 0, 1]])

  # transformation from forearm link to elbow link
  T_FE = np.array([[0, 0, 1, L4x],
                   [m.sin(t4), m.cos(t4), 0, L4z],
                   [-m.cos(t4), m.sin(t4), 0, 0],
                   [0, 0, 0, 1]])

  # transformation from wrist link to forearm link
  T_WF = np.array([[0, 0, -1,0],
                   [m.sin(t5), m.cos(t5), 0, 0],
                   [m.cos(t5), -m.sin(t5), 0, L5],
                   [0, 0, 0, 1]])

  # transformation from hand link to wrist link
  T_HW = np.array([[0, 0, 1, L6x],
                   [m.sin(t6), m.cos(t6), 0, -L6z],
                   [-m.cos(t6), m.sin(t6), 0, 0],
                   [0, 0, 0, 1]])

  # transformation from tool link to hand link
  T_TH = np.array([[0, 1, 0, 0],
                   [0, 0, 1, 0],
                   [1, 0, 0, L7],
                   [0, 0, 0, 1]])

  # transformation from tool link to base link
  T_TB = T_SB.dot(T_AS).dot(T_EA).dot(T_FE).dot(T_WF).dot(T_HW)

  T_03 = T_SB.dot(T_AS).dot(T_EA)

  T_36 = np.linalg.inv(T_03).dot(T_TB)

  # position of origin of tool link in tool link referential
  TL_pos = np.array([0, 0, 0]).T

  # tool link position in world/base link referential
  TL_pos_world = homogeneous_to_cartesian(np.matmul(T_TB, cartesian_to_homogeneous(TL_pos)))

  return TL_pos, TL_pos_world

def inverse_kinematics(L1, L2, L3, L4x, L4z, L5, L6x, L6z, L7, TL_pos_world):
  # vector for the joint angles
  t = [0, 0, 0, 0, 0, 0]

  # compute t1
  t[0] = compute_t1(TL_pos_world)

  # transform desired position from world referential to shoulder referential
  TL_pos_shoulder = world_to_shoulder(t[0], L1, TL_pos_world)

  # compute hand link position in shoulder referential
  HL_pos_shoulder = compute_HL_pos(L7, TL_pos_shoulder)

  # compute wrist link position in shoulder referential
  WL_pos_shoulder = compute_WL_pos(L6x, L6z, HL_pos_shoulder)

  # compute angles 2, 3, 4, 5 and 6
  t[1], t[2], t[3], t[4], t[5] = compute_t2_to_t6(L2, L3, L4x, L4z, L5, WL_pos_shoulder, t[0])

  return t

def qrcodedetector(frame):
	for barcode in decode(frame):
		myData = barcode.data.decode('utf-8')
		points = np.array([barcode.polygon],np.int32)
		points = points.reshape((-1,1,2))
		cv2.polylines(frame,[points],True,(255,0,255),5)
		pts2 = barcode.rect
		ref = pts2.height
		cv2.putText(frame,myData,(pts2[0],pts2[1]),cv2.FONT_HERSHEY_SIMPLEX, 0.9,(255,0,255),2)
		return ref

def arUcodetector(frame):
	aux_top_right = []
	aux_top_left = []
	aux_bottom_right = []
	aux_bottom_left = []
	detected = 0
	gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
	aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
	parameters =  aruco.DetectorParameters_create()
	corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
	frame_markers = aruco.drawDetectedMarkers(frame.copy(), corners, ids)
	auxiliar = 2

	#print(ids)

	if len(corners) != 0:
		for i in range(len(ids)):
			c = corners[i][0]

			plt.plot([c[:, 0].mean()], [c[:, 1].mean()], "o", label = "id={0}".format(ids[i]))

	# verify *at least* one ArUco marker was detected
	if len(corners) > 0:
		# flatten the ArUco IDs list
		ids = ids.flatten()

		# loop over the detected ArUCo corners
		for (markerCorner, markerID) in zip(corners, ids):
			# extract the marker corners (which are always returned
			# in top-left, top-right, bottom-right, and bottom-left
			# order)

			detected = detected + 1 
			corners = markerCorner.reshape((4, 2))
			(topLeft, topRight, bottomRight, bottomLeft) = corners

			# convert each of the (x, y)-coordinate pairs to integers
			topRight = (int(topRight[0]), int(topRight[1]))
			bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
			bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
			topLeft = (int(topLeft[0]), int(topLeft[1]))

			aresta = math.sqrt((bottomRight[0] - topRight[0]) ** 2 + (bottomRight[1] - topRight[1]) ** 2)

			aux_top_right.append(topRight)
			aux_top_left.append(topLeft)
			aux_bottom_right.append(bottomRight)
			aux_bottom_left.append(bottomLeft)

			# draw the bounding box of the ArUCo detection
			cv2.line(frame, topLeft, topRight, (0, 255, 0), 2)
			cv2.line(frame, topRight, bottomRight, (0, 255, 0), 2)
			cv2.line(frame, bottomRight, bottomLeft, (0, 255, 0), 2)
			cv2.line(frame, bottomLeft, topLeft, (0, 255, 0), 2)

			# compute and draw the center (x, y)-coordinates of the
			# ArUco marker
			cX = int((topLeft[0] + bottomRight[0]) / 2.0)
			cY = int((topLeft[1] + bottomRight[1]) / 2.0)
			centro = (cX, cY)

			cv2.circle(frame, (cX, cY), 4, (0, 0, 255), -1)

			# draw the ArUco marker ID on the frame
			cv2.putText(frame, str(markerID),
				(topLeft[0], topLeft[1] - 15),
				cv2.FONT_HERSHEY_SIMPLEX,
				0.5, (0, 255, 0), 2)
		
		return aresta, centro, aux_top_right, aux_top_left, aux_bottom_right, aux_bottom_left, detected
	return auxiliar, auxiliar, auxiliar, auxiliar, auxiliar, auxiliar
			
			
def speedcalc(center_current, center_previous, ball_diameter, diameter, deltatime):
	distance_pix = math.sqrt((center_current[0] - center_previous[0]) ** 2 + (center_current[1] - center_previous[1]) ** 2)

	dx = center_current[0] - center_previous[0]
	dy = center_current[1] - center_previous[1]
	vX = dx/deltatime
	vY = dy/deltatime

	#from pixel to meter
	norm = (ball_diameter * distance_pix)/diameter
	#velocity
	velocity = norm/deltatime
	return norm, velocity, dx, dy, vX, vY, distance_pix

def calcpredict(pos_current, deltatime, velocity_ball, deslocamento):

	aux = (1,1)
	pos_next.append(pos_current)

	print(pos_next)
	for i in range(1,10):
		pos_next.append(pos_next[i-1] + deslocamento)

	return pos_next

def calc_dif_y(dif_x, hipotenusa):
	teta_goal = math.cos(dif_x/hipotenusa)
	dif_y = math.asin(teta_goal)*hipotenusa

	return dif_y

if __name__ == "__main__":

	rospy.init_node("camera_ball")
	rate = rospy.Rate(10)

	# Connecting to robot
  	niryo_one_client = NiryoOneClient()
  	niryo_one_client.connect("169.254.200.200")  # =< Replace by robot ip address 10.2.0.104

	# Trying to calibrate
	status, data = niryo_one_client.calibrate(CalibrateMode.AUTO)
	if status is False:
		print("Error: " + data)

	niryo_one_client.set_arm_max_velocity(100)

	# Getting pose
	status, pose_array, data = niryo_one_client.get_pose()
	print('pose', data)
	initial_pose = None
	pose = []
	if status is True:
		initial_pose = data
		pose = pose_array
	else:
		print("Error: " + data)
	pose = [pose[0]*1000, pose[1]*1000, pose[2]*1000]
	#print pose
	print("Posicao Inicial",pose)
	#print(initial_pose)

	# Load Kalman filter to predict the trajectory
	kf = KalmanFilter()
	speed = Calc_Speed()

	#variables
	ponto_atual = 0 #current point
	dist = 0 #distance the ball goes in each frame
	times = [] #array of the times for each iteration
	centers = [] #array of the ball's centers in each frame
	center_current = [] #array with the current center
	#dimensions in meters
	ball_diameter = 0.041 #0.0654
	ball_mass = 0.055
	arucosize = 0.0685
	field_length = 0.4
	field_height = 0.55
	n_arucos = 2
	process = 0
	steps_ahead = 15

	# Replace the URL with your own IPwebcam shot.jpg IP:port
	#url = 'http://10.2.0.100:8080//shot.jpg' #'http://192.168.1.111:8080//shot.jpg' #http://192.168.1.137:8080//shot.jpg
	
	url = 'http://192.168.43.233:8080//shot.jpg'
	#url = 'http://192.168.43.1:8080//shot.jpg'
	# construct the argument parse and parse the arguments
	ap = argparse.ArgumentParser()
	ap.add_argument("-v", "--video",
		help="path to the (optional) video file")
	ap.add_argument("-b", "--buffer", type=int, default=64,
		help="max buffer size")
	args = vars(ap.parse_args())

	# define the lower and upper boundaries of the "green"
	# ball in the HSV color space, then initialize the
	# list of tracked points
	greenLower = (29, 86, 6)
	greenUpper = (64, 255, 255)
	pts = deque(maxlen=args["buffer"])
	predicts = deque(maxlen=args["buffer"])

	# if a video path was not supplied, grab the reference
	# to the webcam
	if not args.get("video", False):
		vs = VideoStream(src=0).start()

	# otherwise, grab a reference to the video file
	else:
		vs = cv2.VideoCapture(args["video"])

	# allow the camera or video file to warm up
	time.sleep(1.0)

	# keep looping
	while not rospy.is_shutdown():
		
		# grab the current frame
		frame = vs.read()
		starttime = time.time()

		# handle the frame from VideoCapture or VideoStream
		if args.get("video", False):
			init_time = datetime.datetime.now()
    		# Use urllib to get the image from the IP camera
    		imgResponse = urllib.request.urlopen(url)
    		# Numpy to convert into a array
        	imgNp = np.array(bytearray(imgResponse.read()),dtype=np.uint8)
    		# Decode the array to OpenCV usable format
    		frame = cv2.imdecode(imgNp,-1)

		#frame = frame[1] if args.get("video", False) else frame
		# if we are viewing a video and we did not grab a frame,
		# then we have reached the (181, 196)

		# resize the frame, blur it, and convert it to the HSV
		# color space
		frame = imutils.resize(frame, width=600)
		blurred = cv2.GaussianBlur(frame, (11, 11), 0)
		hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

		# construct a mask for the color "green", then perform
		# a series of dilations and erosions to remove any small
		# blobs left in the mask
		mask = cv2.inRange(hsv, greenLower, greenUpper)
		mask = cv2.erode(mask, None, iterations=2)
		mask = cv2.dilate(mask, None, iterations=2)

		# find contours in the mask and initialize the current
		# (x, y) center of the ball
		cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
			cv2.CHAIN_APPROX_SIMPLE)
		cnts = imutils.grab_contours(cnts)
		center = None
		predicted = None

		#Detect and obtain information about the QRCode
		qrcoderef = qrcodedetector(frame)
		arucopixel, aruco_center, aux_top_right, aux_top_left, aux_bottom_right, aux_bottom_left, detected  = arUcodetector(frame)
		#print(detected)
		#detected = 2
		#Getting the field limits (aux_-_- [which aruco][coordinate])
		if detected == n_arucos:
			max_y_top = aux_top_right[1][1]
			max_y_bottom = aux_bottom_left[0][1]
			max_x_left = aux_bottom_left[1][0]
			max_x_right = aux_top_right[0][0]
			print(max_x_right)
			print(max_x_left)
		else:
			print('Arucos not detected')
		#3 Simple rule to find length and height in pixel
		field_length_pixel = (arucopixel * field_length)/arucosize
		field_height_pixel = (arucopixel * field_height)/arucosize

		# only proceed if at least one contour was found
		if len(cnts) > 0:
			# find the largest contour in the mask, then use
			# it to compute the minimum enclosing circle and
			# centroid
			c = max(cnts, key=cv2.contourArea)
			((x, y), radius) = cv2.minEnclosingCircle(c)
			diameter = 2*radius
			M = cv2.moments(c)
			center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

			#calculate the prediction with Kalman Filter
			predicted = kf.predict(center[0], center[1])

			# only proceed if the radius meets a minimum size
			if radius > 10:
				# draw the circle and centroid on the frame,
				# then update the list of tracked points
				cv2.circle(frame, (int(x), int(y)), int(radius),
					(0, 0, 255), 2)
				cv2.circle(frame, center, 5, (0, 255, 0), -1) #contour of the center of the ball
				cv2.circle(frame, (predicted[0], predicted[1]), 5, (255, 0, 0), 4) # blue contour of the center of the prediction

		# update the points queue
		pts.appendleft(center)
		centers.append(center)
		predicts.appendleft(predicted)

		# loop over the set of tracked points
		for i in range(1, len(pts)):

			# if either of the tracked points are None, ignore
			# them
			if (pts[i - 1] and predicts[i-1]) is None or (pts[i] and predicts[i]) is None:
				continue

			# otherwise, compute the thickness of the line and
			# draw the connecting lines
			thickness = int(np.sqrt(args["buffer"] / float(i + 1)) * 2.5)
			#cv2.line(frame, pts[i - 1], pts[i], (0, 255, 0), thickness)
			#cv2.line(frame, predicts[i - 1], predicts[i], (255, 0, 0), thickness)

		endtime = time.time()

		#Calculate elapsed time for this iteration
		times.append(endtime-starttime)
		deltatime = times[ponto_atual]

		#Calculate distance and velocity ran during this iteration
		center_current = centers[ponto_atual]
		center_previous = centers[ponto_atual - 1]
		pos_next = []
		pos_next_reflected = []
		x_next = []
		y_next = []
		x_next_reflected = []
		y_next_reflected = []
		aux = []
		aux_reflected = []
		counter = 0
		reflection = 0
		goal_point = []
		goal = 0


		if center_current != None and center_previous != None and ponto_atual != 0 and arucopixel != None and aruco_center != 2 and process%5 == 0:
			distance_ball, velocity_ball, dX, dY, vX, vY, distance_pix = speedcalc(center_current, center_previous, arucosize, arucopixel, deltatime)
			deslocamento = np.array([dX, dY])

			if distance_pix > 12:
				steps_ahead = 15
			else:
				steps_ahead = 10

			print("deslocamento",distance_pix)
			
			velocity_array = np.array([vX, vY])

			momentum = ball_mass*velocity_array

			x_next.append(center_current[0] + dX)
			y_next.append(center_current[1] + dY)

			for i in range(1,steps_ahead):
				x_next.append(x_next[i-1] + dX)
				y_next.append(y_next[i-1] + dY)
				aux = (x_next[i], y_next[i])
				pos_next.append(aux)

				margin_left = pos_next[i-1][0] - max_x_left
				margin_right = pos_next[i-1][0] - max_x_right

				if pos_next[i-1][0] <= max_x_left and abs(margin_left) < 5 and distance_pix >= 3:
					goal = 1
					#hipotenusa = math.sqrt(([0] - center_previous[0]) ** 2 + (center_current[1] - center_previous[1]) ** 2)
					goal_point_X = max_x_left
					goal_point_Y = pos_next[i-1][1]
					goal_point = (goal_point_X, goal_point_Y)

				if pos_next[i-1][0] >= max_x_right and abs(margin_right) < 5 and distance_pix >= 3:
					goal = 1
					goal_point_X = max_x_right
					goal_point_Y = pos_next[i-1][1]
					goal_point = (goal_point_X, goal_point_Y)

			#Detect if the ball hits a wall and predict the reflected trajectory
			#Assumptions: 
			#1) The ball does not lose velocity 
			#2) The incident angle is the same as the reflected angle
			for i in range(1,steps_ahead):
				if pos_next[i-1][1] >= max_y_bottom and distance_pix >= 3:
					#print("passou em baixo")
					reflection = 1 
					x_next_reflected.append(pos_next[i-1][0] + dX)
					y_next_reflected.append(pos_next[i-1][1] + dY)
					last_pos_ref = pos_next[i - 1]

					for j in range(1,steps_ahead):				
						x_next_reflected.append(x_next_reflected[j-1] + dX)
						y_next_reflected.append(y_next_reflected[j-1] - dY)
						aux_reflected = (x_next_reflected[j], y_next_reflected[j])
						pos_next_reflected.append(aux_reflected)
						#cv2.circle(frame, pos_next_reflected[j-1], 5, (0, 255, 0), -1) #contour of the center of the ball

						margin_left_reflected = pos_next[i-1][0] - max_x_left
						margin_right_reflected = pos_next[i-1][0] - max_x_right

						if pos_next_reflected[j-1][0] <= max_x_left and abs(margin_left_reflected) < 5 and distance_pix >= 3: #distance_pix is the run distance in pixel 
							#print("passou esquerda")
							goal = 1
							goal_point_X = max_x_left
							goal_point_Y = pos_next_reflected[j-1][1]
							goal_point = (goal_point_X, goal_point_Y)
							#cv2.circle(frame, goal_point, 5, (100, 255, 100), -1) #contour of the center of the ball
						if pos_next_reflected[j-1][0] >= max_x_right and abs(margin_right_reflected) < 5 and distance_pix >= 3:
							#print("passou direita")
							goal = 1
							goal_point_X = max_x_right
							goal_point_Y = pos_next_reflected[j-1][1]
							goal_point = (goal_point_X, goal_point_Y)
							#cv2.circle(frame, goal_point, 5, (100, 255, 100), -1) #contour of the center of the ball

				if pos_next[i-1][1] <= max_y_top and distance_pix >= 3:
					#print("passou em cima")
					reflection = 1
					x_next_reflected.append(pos_next[i-1][0] + dX)
					y_next_reflected.append(pos_next[i-1][1] + dY)
					last_pos_ref = pos_next[i - 1]

					for j in range(1,steps_ahead):
						x_next_reflected.append(x_next_reflected[j-1] + dX)
						y_next_reflected.append(y_next_reflected[j-1] - dY)
						aux_reflected = (x_next_reflected[j], y_next_reflected[j])
						pos_next_reflected.append(aux_reflected)
						#cv2.circle(frame, pos_next_reflected[j-1], 5, (0, 255, 0), -1) #contour of the center of the ball

						margin_left_reflected = pos_next[i-1][0] - max_x_left
						margin_right_reflected = pos_next[i-1][0] - max_x_right


						if pos_next_reflected[j-1][0] <= max_x_left and abs(margin_left_reflected) < 5 and distance_pix >= 3:
							#print("passou esquerda")
							goal = 1
							goal_point_X = max_x_left
							goal_point_Y = pos_next_reflected[j-1][1]
							goal_point = (goal_point_X, goal_point_Y)
							#cv2.circle(frame, goal_point, 5, (100, 255, 100), -1) #contour of the center of the ball
						if pos_next_reflected[j-1][0] >= max_x_right and abs(margin_right_reflected) < 5 and distance_pix >= 3:
							#print("passou direita")
							goal = 1 
							goal_point_X = max_x_right
							goal_point_Y = pos_next_reflected[j-1][1]
							goal_point = (goal_point_X, goal_point_Y)
							#cv2.circle(frame, goal_point, 5, (100, 255, 100), -1) #contour of the center of the ball

			if reflection == 0:
				for i in range(1,steps_ahead-1):
					cv2.line(frame, pos_next[i - 1], pos_next[i], (255, 0, 0), thickness)
					#cv2.line(frame,last_pos_ref, pos_next_reflected[i], (255, 5, 0), thickness)

				if goal == 1:
					shift_alongsideY = goal_point_Y - 240 #center of the arm corresponds to 240 pixel in the Y coordinate 
					pose_X_robot = (shift_alongsideY * 225)/122 #3 simple rule, 255   
					print("shift_alongsideY",shift_alongsideY)
					print("pose_X_robot",pose_X_robot)

					if abs(pose_X_robot) > 265:
						pose_X_robot = 265

					cv2.circle(frame, goal_point, 5, (255, 255, 100), -1) #contour of the center of the ball
					#print(goal_point)

					goal = [200, pose_X_robot, 70]
					goal = np.array(goal)
					print("Posicao Objetivo", goal)

					# Getting joints
					status, data = niryo_one_client.get_joints()
					initial_joints = None
					if status is True:
						initial_joints = data
					else:
						print("Error: " + data)
					print("Angulos Iniciais", initial_joints)


					#INVERSE KINEMATICS
					[L1, L2, L3, L4x, L4z, L5, L6x, L6z, L7] = [103, 80, 210, 41.5, 30, 180, 23.7, 5.5, 0]
					angles = inverse_kinematics(L1, L2, L3, L4x, L4z, L5, L6x, L6z, L7, goal)
					print("Resultado do Inverse Kinematics",angles)


					#DIRECT KINEMATICS
					L = [103, 80, 210, 41.5, 30, 180, 23.7, 5.5, 0]
					TL_pos_2, resultado2 = direct_kinematics(angles, L)
					print("Resultado do Direct Kinematics", resultado2)


					# Move Joints
					status, data = niryo_one_client.move_joints(angles[0], angles[1], angles[2], angles[3], angles[4], angles[5])
					if status is False:
						print("Error: " + data)
				
			if reflection == 1:
				for i in range(1,steps_ahead-1):
					#cv2.line(frame, pos_next[i - 1], pos_next[i], (255, 0, 0), thickness)
					cv2.line(frame,pos_next_reflected[i-1], pos_next_reflected[i], (255, 5, 0), thickness)

				if goal == 1:

					shift_alongsideY = goal_point_Y - 240 #center of the arm corresponds to 240 pixel in the Y coordinate 
					pose_X_robot = (shift_alongsideY * 225)/122 #3 simple rule, 255   
					print("shift_alongsideY",shift_alongsideY)
					print("pose_X_robot",pose_X_robot)

					if abs(pose_X_robot) > 265:
						pose_X_robot = 265


					cv2.circle(frame, goal_point, 5, (255, 255, 100), -1) #contour of the center of the ball
					#print(goal_point)


					goal = [200, pose_X_robot, 70]
					goal = np.array(goal)
					print("Posicao Objetivo", goal)

					# Getting joints
					status, data = niryo_one_client.get_joints()
					initial_joints = None
					if status is True:
						initial_joints = data
					else:
						print("Error: " + data)
					print("Angulos Iniciais", initial_joints)

					#INVERSE KINEMATICS
					[L1, L2, L3, L4x, L4z, L5, L6x, L6z, L7] = [103, 80, 210, 41.5, 30, 180, 23.7, 5.5, 0]
					angles = inverse_kinematics(L1, L2, L3, L4x, L4z, L5, L6x, L6z, L7, goal)
					print("Resultado do Inverse Kinematics",angles)

					#DIRECT KINEMATICS
					L = [103, 80, 210, 41.5, 30, 180, 23.7, 5.5, 0]
					TL_pos_2, resultado2 = direct_kinematics(angles, L)
					print("Resultado do Direct Kinematics", resultado2)

					# Move Joints
					status, data = niryo_one_client.move_joints(angles[0], angles[1], angles[2], angles[3], angles[4], angles[5])
					if status is False:
						print("Error: " + data)
				
		reflection = 0
		#increment current point
		ponto_atual += 1

		process = process + 1


		# show the frame to our screen
		cv2.imshow("Frame", frame)
		key = cv2.waitKey(1) & 0xFF

		# if the 'q' key is pressed, stop the loop
		if key == ord("q"):
			break

		rate.sleep()

	# if we are not using a video file, stop the camera video stream
	if not args.get("video", False):
		vs.stop()

	# otherwise, release the camera
	else:
		vs.release()

	# close all windows
	cv2.destroyAllWindows()

		