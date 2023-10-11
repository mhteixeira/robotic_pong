
import cv2
import numpy as np
from helpers import detect_ball
import time
import matplotlib.pyplot as plt
from params import *

# Create a VideoCapture object and read from input file
# If the input is the camera, pass 0 instead of the video file name
video_file_path = './tests_for_components/ball_velocity/ball_velocity.mp4'
cap = cv2.VideoCapture(video_file_path)
 
# Check if camera opened successfully
if (cap.isOpened()== False): 
	print("Error opening video stream or file")

previous_x, previous_y = 0, 0
previous_time = 0
counter = 1
dx_array = []
dy_array = []
dt_array = []
times = []
pixel_density = 0
arucos_identified = False

# Configuring the ArUco detector
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
parameters =  cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)
was_ball_detected = False

def sign(x):
	if x >= 0: return 1
	else: return -1

vel = 0

# Read until video is completed
while(cap.isOpened()):
	# Capture frame-by-frame
	try:
		ret, frame = cap.read()
		frame = frame[60:-80, :-80]
		if ret == True:

			if not arucos_identified:
				# First we make the image monochromatic
				gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

				# Then we detect the arucos
				corners, ids, rejectedImgPoints = detector.detectMarkers(gray)
				frame = cv2.aruco.drawDetectedMarkers(frame.copy(), corners, ids)
				corners = np.array(corners, dtype=int)
				if len(corners > 0):
					# print(corners)
					# The bottom right corner is identified by the Aruco with ID = 2
					bottom_right_corner_id = np.where(ids == 2)[0]

					# The bottom right corner is identified by the Aruco with ID = 3
					top_right_corner_id = np.where(ids == 3)[0]

					bottom_right_corner = corners[bottom_right_corner_id][0][0][0]
					top_right_corner = corners[top_right_corner_id][0][0][0]
					print(top_right_corner[1] - bottom_right_corner[1])
					pixel_density = abs(top_right_corner[1] - bottom_right_corner[1])/field_lenght_m
					print('Pixel density (pixels/m)', pixel_density)
					arucos_identified = True
			else:
				# Display the resulting frame
				is_ball_detected, _, _, x, y, radius = detect_ball(frame, frame)
				if is_ball_detected:
					if was_ball_detected == False:
						current_time = time.time()
						previous_x = x
						previous_y = y
						previous_time = current_time
					else:
						if counter >= 6:
							current_time = time.time()
							# print("dt:", current_time - previous_time)
							# print("dx:", (x - previous_x)/pixel_density)
							dx = x - previous_x
							dy = y - previous_y
							dt = current_time - previous_time
							dx_array.append(dx)
							dy_array.append(dy)
							dt_array.append(dt)
							times.append(current_time)
							if dt > 0.001:
								vel = sign(dx)*np.hypot(dx, dy)/pixel_density/dt
								print("vel:", vel)
							print()
							previous_x = x
							previous_y = y
							previous_time = current_time
							counter = 1
						counter += 1
				was_ball_detected = is_ball_detected
				cv2.putText(frame, f'{vel:.2f}', (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 
                   1, (255, 255, 255), 2, cv2.LINE_AA)
				cv2.putText(frame, f'{vel:.2f}', (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 
                   1, (0, 0, 0), 1, cv2.LINE_AA)
				cv2.imshow('Frame',frame)
				# Press Q on keyboard to  exit
				if cv2.waitKey(25) & 0xFF == ord('q'):
					break

		# Break the loop
		else: 
			break
	except Exception as e: 
		print(e)
		break
 
# When everything done, release the video capture object
cap.release()
 
# Closes all the frames
cv2.destroyAllWindows()

dx_array = np.array(dx_array)
dy_array = np.array(dy_array)
dt_array = np.array(dt_array)
vel = [sign(dx)*np.hypot(dx, dy)/pixel_density/dt for dx, dy, dt in zip(dx_array, dy_array, dt_array)]
times = np.array(times) - times[0]
plt.plot(times[10:], vel[10:])
plt.title("Velocidade da bola durante ensaio")
plt.xlabel("Tempo (s)")
plt.ylabel("Velocidade (m/s)")
plt.grid()
# plt.ylim([-7, 7])
plt.show()