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
import cv2.aruco as aruco
import matplotlib.pyplot as plt
import matplotlib as mpl
import numpy as np
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
		detected = detected + 1 


		# loop over the detected ArUCo corners
		for (markerCorner, markerID) in zip(corners, ids):
			# extract the marker corners (which are always returned
			# in top-left, top-right, bottom-right, and bottom-left
			# order)
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
	rate = rospy.Rate(2)

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

	steps_ahead = 7

	# Replace the URL with your own IPwebcam shot.jpg IP:port
	url='http://192.168.43.1:8080//shot.jpg' #http://192.168.1.137:8080//shot.jpg

	# construct the argument parse and parse the arguments
	ap = argparse.ArgumentParser()
	ap.add_argument("-v", "--video",
		help="path to the (optional) video file")
	ap.add_argument("-b", "--buffer", type=int, default=64,
		help="max buffer size")
	args = vars(ap.parse_args())

	print(args)

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

		print("1")

		# handle the frame from VideoCapture or VideoStream
		if args.get("video", False):
			print("inside")
			init_time = datetime.datetime.now()
			# Use urllib to get the image from the IP camera
			imgResponse = urllib.request.urlopen(url)
			# Numpy to convert into a array
			imgNp = np.array(bytearray(imgResponse.read()),dtype=np.uint8)
			# Decode the array to OpenCV usable format
			frame = cv2.imdecode(imgNp,-1)

		#frame = frame[1] if args.get("video", False) else frame
		print(frame)
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
		detected = 2
		print (detected)
		#Getting the field limits (aux_-_- [which aruco][coordinate])
		if detected == n_arucos:
			max_y_top = aux_top_right[1][1]
			max_y_bottom = aux_bottom_left[0][1]
			max_x_left = aux_bottom_left[1][0]
			max_x_right = aux_top_right[0][0]
			print(max_x_right)
			print(max_x_left)
		

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

		if center_current != None and center_previous != None and ponto_atual != 0 and arucopixel != None and aruco_center != 2:
			distance_ball, velocity_ball, dX, dY, vX, vY, distance_pix = speedcalc(center_current, center_previous, arucosize, arucopixel, deltatime)
			deslocamento = np.array([dX, dY])

			print("deslocamento",distance_pix)

			velocity_array = np.array([vX, vY])

			momentum = ball_mass*velocity_array

			x_next.append(center_current[0] + dX)
			y_next.append(center_current[1] + dY)

			for i in range(1,10):
				x_next.append(x_next[i-1] + dX)
				y_next.append(y_next[i-1] + dY)
				aux = (x_next[i], y_next[i])
				pos_next.append(aux)

				margin_left = pos_next[i-1][0] - max_x_left
				margin_right = pos_next[i-1][0] - max_x_right

				if pos_next[i-1][0] <= max_x_left and abs(margin_left) < 5 and distance_pix >= 3 and detected == n_arucos:
					goal = 1
					#hipotenusa = math.sqrt(([0] - center_previous[0]) ** 2 + (center_current[1] - center_previous[1]) ** 2)
					goal_point_X = max_x_left
					goal_point_Y = pos_next[i-1][1]
					goal_point = (goal_point_X, goal_point_Y)

				if pos_next[i-1][0] >= max_x_right and abs(margin_right) < 5 and distance_pix >= 3 and detected == n_arucos:
					goal = 1
					goal_point_X = max_x_right
					goal_point_Y = pos_next[i-1][1]
					goal_point = (goal_point_X, goal_point_Y)

			#Detect if the ball hits a wall and predict the reflected trajectory
			#Assumptions:
			#1) The ball does not lose velocity
			#2) The incident angle is the same as the reflected angle
			for i in range(1,10):
				if pos_next[i-1][1] >= max_y_bottom and distance_pix >= 3 and detected == n_arucos:
					print("passou em baixo")
					reflection = 1 
					x_next_reflected.append(pos_next[i-1][0] + dX)
					y_next_reflected.append(pos_next[i-1][1] + dY)
					last_pos_ref = pos_next[i - 1]

					for j in range(1,10):				
						x_next_reflected.append(x_next_reflected[j-1] + dX)
						y_next_reflected.append(y_next_reflected[j-1] - dY)
						aux_reflected = (x_next_reflected[j], y_next_reflected[j])
						pos_next_reflected.append(aux_reflected)
						#cv2.circle(frame, pos_next_reflected[j-1], 5, (0, 255, 0), -1) #contour of the center of the ball

						margin_left_reflected = pos_next[i-1][0] - max_x_left
						margin_right_reflected = pos_next[i-1][0] - max_x_right

						if pos_next_reflected[j-1][0] <= max_x_left and abs(margin_left_reflected) < 5 and distance_pix >= 3 and detected == n_arucos: #distance_pix is the run distance in pixel 
							print("passou esquerda")
							goal = 1
							goal_point_X = max_x_left
							goal_point_Y = pos_next_reflected[j-1][1]
							goal_point = (goal_point_X, goal_point_Y)
							#cv2.circle(frame, goal_point, 5, (100, 255, 100), -1) #contour of the center of the ball
						if pos_next_reflected[j-1][0] >= max_x_right and abs(margin_right_reflected) < 5 and distance_pix >= 3 and detected == n_arucos:
							print("passou direita")
							goal = 1
							goal_point_X = max_x_right
							goal_point_Y = pos_next_reflected[j-1][1]
							goal_point = (goal_point_X, goal_point_Y)
							#cv2.circle(frame, goal_point, 5, (100, 255, 100), -1) #contour of the center of the ball

				if pos_next[i-1][1] <= max_y_top and distance_pix >= 3 and detected == n_arucos:
					print("passou em cima")
					reflection = 1
					x_next_reflected.append(pos_next[i-1][0] + dX)
					y_next_reflected.append(pos_next[i-1][1] + dY)
					last_pos_ref = pos_next[i - 1]

					for j in range(1,10):
						x_next_reflected.append(x_next_reflected[j-1] + dX)
						y_next_reflected.append(y_next_reflected[j-1] - dY)
						aux_reflected = (x_next_reflected[j], y_next_reflected[j])
						pos_next_reflected.append(aux_reflected)
						#cv2.circle(frame, pos_next_reflected[j-1], 5, (0, 255, 0), -1) #contour of the center of the ball

						margin_left_reflected = pos_next[i-1][0] - max_x_left
						margin_right_reflected = pos_next[i-1][0] - max_x_right


						if pos_next_reflected[j-1][0] <= max_x_left and abs(margin_left_reflected) < 5 and distance_pix >= 3 and detected == n_arucos:
							print("passou esquerda")
							goal = 1
							goal_point_X = max_x_left
							goal_point_Y = pos_next_reflected[j-1][1]
							goal_point = (goal_point_X, goal_point_Y)
							#cv2.circle(frame, goal_point, 5, (100, 255, 100), -1) #contour of the center of the ball
						if pos_next_reflected[j-1][0] >= max_x_right and abs(margin_right_reflected) < 5 and distance_pix >= 3 and detected == n_arucos:
							print("passou direita")
							goal = 1 
							goal_point_X = max_x_right
							goal_point_Y = pos_next_reflected[j-1][1]
							goal_point = (goal_point_X, goal_point_Y)
							#cv2.circle(frame, goal_point, 5, (100, 255, 100), -1) #contour of the center of the ball
					
			if reflection == 0:
				for i in range(1,9):
					cv2.line(frame, pos_next[i - 1], pos_next[i], (255, 0, 0), thickness)
					#cv2.line(frame,last_pos_ref, pos_next_reflected[i], (255, 5, 0), thickness)

				if goal == 1:
					cv2.circle(frame, goal_point, 5, (255, 255, 100), -1) #contour of the center of the ball
				
			if reflection == 1:
				for i in range(1,9):
					#cv2.line(frame, pos_next[i - 1], pos_next[i], (255, 0, 0), thickness)
					cv2.line(frame,pos_next_reflected[i-1], pos_next_reflected[i], (255, 5, 0), thickness)

				if goal == 1:
					cv2.circle(frame, goal_point, 5, (255, 255, 100), -1) #contour of the center of the ball
				
		reflection = 0
		#increment current point
		ponto_atual += 1

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

		