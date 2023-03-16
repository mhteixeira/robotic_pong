#!/usr/bin/env python


# python ball_tracking.py --video ball_tracking_example.mp4


# import the necessary packages
from collections import deque
from email import message
from turtle import distance
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
from std_msgs.msg import Float32
from std_msgs.msg import Int32
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
	return auxiliar, auxiliar, auxiliar, auxiliar, auxiliar, auxiliar, 0
			
			
def speedcalc(center_current, center_previous, ball_diameter, diameter, deltatime, max_x_right):
	distance_pix = math.sqrt((center_current[0] - center_previous[0]) ** 2 + (center_current[1] - center_previous[1]) ** 2)
	distance_to_arm = abs(max_x_right - center_current[0])
	#print (distance_to_arm)
	dx = center_current[0] - center_previous[0]
	dy = center_current[1] - center_previous[1]
	vX = dx/deltatime
	vY = dy/deltatime

	#from pixel to meter
	norm = (ball_diameter * distance_pix)/diameter
	#velocity
	velocity = norm/deltatime
	return norm, velocity, dx, dy, vX, vY, distance_pix, distance_to_arm

def calcpredict(pos_current, deltatime, velocity_ball, deslocamento):

	aux = (1,1)
	pos_next.append(pos_current)

	for i in range(1,10):
		pos_next.append(pos_next[i-1] + deslocamento)

	return pos_next

def calc_dif_y(dif_x, hipotenusa):
	teta_goal = math.cos(dif_x/hipotenusa)
	dif_y = math.asin(teta_goal)*hipotenusa

	return dif_y

if __name__ == "__main__":


	send_pose = rospy.Publisher('goal_pose', Float32, queue_size=10)
	send_kick = rospy.Publisher('kick_state', Int32, queue_size=10)
	send_flag_pose = rospy.Publisher('flag_pose', Int32, queue_size=10)
	rospy.init_node("camera_ball")
	rate = rospy.Rate(20)

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
	#field_length = 0.4
	#field_height = 0.55
	n_arucos = 2
	process = 0
	steps_ahead = 15
	enviar = 1
	message_sent = 0

	# Replace the URL with your own IPwebcam shot.jpg IP:port
	#url= 'http://10.2.0.107:8080//shot.jpg' #'http://192.168.1.111:8080//shot.jpg' #http://192.168.1.137:8080//shot.jpg
	url = 'http://192.168.43.233:8080//shot.jpg'

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

		imgResponse = urllib.request.urlopen(url)
		imgNp = np.array(bytearray(imgResponse.read()),dtype=np.uint8)
		frame = cv2.imdecode(imgNp,-1)
		
		# handle the frame from VideoCapture or VideoStream
		if args.get("video", False):
			init_time = datetime.datetime.now()
    		# Use urllib to get the image from the IP camera
    		# Numpy to convert into a array
    		# Decode the array to OpenCV usable format

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
		#detected = 2
		#Getting the field limits (aux_-_- [which aruco][coordinate])
		if detected == n_arucos:
			max_y_top = aux_top_right[1][1]
			max_y_bottom = aux_bottom_left[0][1]
			field_height = abs(max_y_top - max_y_bottom)
			max_x_left = aux_bottom_left[1][0]
			max_x_right = aux_top_right[0][0]

		#3 Simple rule to find length and height in pixel
		#field_length_pixel = (arucopixel * field_length)/arucosize
		#field_height_pixel = (arucopixel * field_height)/arucosize

		# only proceed if at least one contour was found
		if len(cnts) > 0:
			# find the largest contour in the mask, then use
			# it to compute the minimum enclosing circle and
			# centroid
			c = max(cnts, key=cv2.contourArea)
			print(cnts[0][0])

			center_trial = cnts[0][0]
			
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
			#thickness = int(np.sqrt(args["buffer"] / float(i + 1)) * 2.5)
			#cv2.line(frame, pts[i - 1], pts[i], (0, 255, 0), thickness)
			#cv2.line(frame, predicts[i - 1], predicts[i], (255, 0, 0), thickness)

		endtime = time.time()

		#Calculate elapsed time for this iteration
		times.append(endtime-starttime)
		deltatime = times[ponto_atual]

		#Calculate distance and velocity ran during this iteration
		
		center_current = centers[ponto_atual]
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
		inside = 0

		cv2.line(frame, (140,0), (140, 450), (255, 0, 0), 3)

		if center_current != None and ponto_atual != 0 and arucopixel != None and aruco_center != 2 and n_arucos == detected:
			flag_pose = 0

			#deslocamento = np.array([dX, dY])
			#velocity_array = np.array([vX, vY])
			#momentum = ball_mass*velocity_array

			#x_next.append(center_current[0] + dX)
			#y_next.append(center_current[1] + dY)
			center_previous = centers[ponto_atual-3]

			if (ponto_atual) % 1 == 0 and center_previous != None and center_current != None:
				

				distance_ball, velocity_ball, dX, dY, vX, vY, distance_pix, distance_to_arm = speedcalc(center_current, center_previous, arucosize, arucopixel, deltatime, max_x_right)

				if (center_current[0]- center_previous[0]) == 0:
					flag_pose = 1
					slope = 0
					b = center_current[1] - slope * center_current[0]
					goal_y = slope*max_x_right + b
					goal_position = (max_x_right, int(goal_y))

					#cv2.circle(frame, goal_position, 5, (255, 255, 100), -1) #contour of the center of the ball
					
				elif (center_current[0]- center_previous[0]) != 0:
					flag_pose = 1
					slope = ((float(center_current[1]) - float(center_previous[1]))/float((center_current[0])- float(center_previous[0])))
					b = center_current[1] - slope * center_current[0]
					goal_y = slope*max_x_right + b
					number_of_fields = math.floor(abs(goal_y)/field_height)
					print("goal_y", goal_y)
					if goal_y <= max_y_top:
						if number_of_fields % 2 == 0:
							goal_y_reflected = field_height -(abs(goal_y) - number_of_fields * field_height)
							print("goal_y_reflected",goal_y_reflected)
						else:
							goal_y_reflected = abs(goal_y) - number_of_fields * field_height
							print("goal_y_reflected_impar",goal_y_reflected)
						goal_y = goal_y_reflected

					elif goal_y >= max_y_bottom: 
						if number_of_fields % 2 == 0:
							goal_y_reflected = goal_y - number_of_fields * field_height
						else:
							goal_y_reflected = field_height -(goal_y - number_of_fields * field_height)
						goal_y = goal_y_reflected

						intersect_X = (max_y_bottom - b)/slope
						b_reflection = max_y_bottom + slope*intersect_X
						goal_y = -slope*max_x_right + b_reflection
						
					goal_position = (max_x_right, int(goal_y))
					#cv2.circle(frame, goal_position, 5, (255, 255, 100), -1) #contour of the center of the ball


				if distance_pix != 0 and center_current[0] >= 140:
					message_sent = 1
					print("Message Sent")

					print(goal_position)

					shift_alongsideY = goal_y - 225 #center of the arm corresponds to 240 pixel in the Y coordinate 
					pose_X_robot = (shift_alongsideY * 200)/125 #3 simple rule, 255   
					#print("shift_alongsideY",shift_alongsideY)
					#print("pose_X_robot",pose_X_robot)

					if abs(pose_X_robot) > 250:
						pose_X_robot = 250
					print("Mensagem Enviada", time.ctime(time.time()))
					send_pose.publish(pose_X_robot)

					send_flag_pose.publish(flag_pose)

					#send information if the ball is close to the arm, so that it can kick the ball
					if distance_to_arm <= 90:
						kick_state = 1
						send_kick.publish(kick_state)
					else:
						kick_state = 0
						send_kick.publish(kick_state)
				else: 
					flag_pose = 0
					send_flag_pose.publish(flag_pose)


				
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

		