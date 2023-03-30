import cv2
import numpy as np
from helpers import resize_image, detectBall, is_point_inside_frame, line_limits

cap = cv2.VideoCapture('refactoring_cv/examples/example15.avi')
# cap = cv2.VideoCapture(1)
# Check if camera opened successfully
if (cap.isOpened()== False): 
  print("Error opening video stream or file")
else:
  ret, frame = cap.read()
  frame = resize_image(frame, 60)
  h_frame, w_frame, _ = frame.shape


# Variables
n_arucos = 2
arucosCenter = [[] for _ in range(n_arucos)]
top_left_corner = np.array([])
bot_right_corner = np.array([])
lim_x = 0
lim_y = 0
x_robot_corner = 0

xd_array = []
yd_array = []
y_pred = h_frame/2

# Kalman filter
kf = cv2.KalmanFilter(4, 2)
kf.measurementMatrix = np.array([[1, 0, 0, 0], [0, 1, 0, 0]], np.float32)
kf.transitionMatrix = np.array([[1, 0, 1, 0], [0, 1, 0, 1], [0, 0, 1, 0], [0, 0, 0, 1]], np.float32)

def delimitField(arucos_corners):
  field_limits = []
  arucos_corners = [corner in corner]
  field_limits = min(arucos_corners)

  return field_limits

is_field_delimited = False

while (not is_field_delimited):
  ret, frame = cap.read()
  if ret == True:
    frame = resize_image(frame, 60)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
    parameters =  cv2.aruco.DetectorParameters_create()
    corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
    frame_markers = cv2.aruco.drawDetectedMarkers(frame.copy(), corners, ids)
    corners = np.array(corners, dtype=int)
    if len(corners) == n_arucos:
      is_field_delimited = True
      lower_edge = line_limits(corners[0][0][2:4], frame)
      upper_edge = line_limits(corners[1][0][0:2], frame)
      left_edge = line_limits(corners[0][0][1:3], frame)
      right_edge = line_limits(corners[1][0][[0, 3]], frame)
      frame = cv2.line(frame, lower_edge[0], lower_edge[1], (200, 200, 200), 1)
      frame = cv2.line(frame, upper_edge[0], upper_edge[1], (200, 200, 200), 1)
      frame = cv2.line(frame, left_edge[0], left_edge[1], (200, 200, 200), 1)
      frame = cv2.line(frame, right_edge[0], right_edge[1], (200, 200, 200), 1)
    cv2.imshow('Delimitation',frame)

while(cap.isOpened()):
  ret, frame = cap.read()
  if ret == True:
    frame = resize_image(frame, 60)
    h_frame, w_frame, _ = frame.shape
    cv2.imshow('Original',frame)

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
    parameters =  cv2.aruco.DetectorParameters_create()
    corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
    frame_markers = cv2.aruco.drawDetectedMarkers(frame.copy(), corners, ids)
    corners = np.array(corners, dtype=int)
    
    if len(corners) == n_arucos:
        
        top_left_corner = corners[1][0][0]
        bot_right_corner = corners[0][0][2]
        height_field = top_left_corner[1] - bot_right_corner[1]
        

        # Identificar se está dentro ou fora usando produto vetorial
        x_robot_corner = corners[0][0][2][0]
        
        for i, corner in enumerate(corners):
            arucosCenter[i] = (int(np.mean(corner[0][:, 0])), int(np.mean(corner[0][:, 1])))
    
    _, _, x, y, radius = detectBall(frame)

    # Kalman predictions
    kf.correct(np.array([x, y], dtype=np.float32))
    _, _, xd_pred, yd_pred = kf.predict()
    
    n = 10
    if xd_pred and yd_pred:
      xd_array.append(xd_pred)
      yd_array.append(yd_pred)
    
    if len(xd_array) > 2*n:
      xd = np.mean(xd_array[-n:])
      yd = np.mean(yd_array[-n:])
      slope = round(yd/xd, 2)
      if xd > 0 and (xd > 0.1 or abs(yd) > 0.1):
          y_pred = slope*(x_robot_corner-x) + y
          print(y_pred)

          if y_pred < top_left_corner[1]:

            overshoot_y =(y_pred - top_left_corner[1])
            number_of_refletions = np.ceil(overshoot_y / height_field).astype(int)
            if (number_of_refletions % 2 == 1):
              y_pred = top_left_corner[1] - (overshoot_y % height_field)
            if (number_of_refletions % 2 == 2):
              y_pred = overshoot_y % height_field
            current_x = x
            next_x = x
            current_y = y
            next_y = y
            for n in range(1, number_of_refletions + 1):
              print(n)
              if n % 2 == 1:
                next_x = (top_left_corner[1] - current_y)/slope + current_x
                next_y = top_left_corner[1]
                cv2.line(frame_markers, (int(current_x), int(current_y)), (int(next_x), top_left_corner[1]), (255, 200, 200), 1)     
              if n % 2 == 0:
                next_x = (bot_right_corner[1] - current_y)/slope + current_x
                next_y = bot_right_corner[1]
                image = cv2.line(frame_markers, (int(current_x), int(current_y)), (int(next_x), bot_right_corner[1]), (255, 200, 200), 1)     
              current_x = next_x
              current_y = next_y
            image = cv2.line(frame_markers, (int(current_x), int(current_y)), (int(x_robot_corner), int(y_pred)), (255, 200, 200), 1)     
          if y_pred > top_left_corner[1]:

            overshoot_y =(y_pred - top_left_corner[1])
            number_of_refletions = np.ceil(overshoot_y / height_field).astype(int)
            if (number_of_refletions % 2 == 1):
              y_pred = top_left_corner[1] - (overshoot_y % height_field)
            if (number_of_refletions % 2 == 2):
              y_pred = overshoot_y % height_field
            current_x = x
            next_x = x
            current_y = y
            next_y = y
            for n in range(1, number_of_refletions + 1):
              print(n)
              if n % 2 == 1:
                next_x = (top_left_corner[1] - current_y)/slope + current_x
                next_y = top_left_corner[1]
                cv2.line(frame_markers, (int(current_x), int(current_y)), (int(next_x), top_left_corner[1]), (255, 200, 200), 1)     
              if n % 2 == 0:
                next_x = (bot_right_corner[1] - current_y)/slope + current_x
                next_y = bot_right_corner[1]
                image = cv2.line(frame_markers, (int(current_x), int(current_y)), (int(next_x), bot_right_corner[1]), (255, 200, 200), 1)     
              current_x = next_x
              current_y = next_y
            image = cv2.line(frame_markers, (int(current_x), int(current_y)), (int(x_robot_corner), int(y_pred)), (255, 200, 200), 1)     
          else:
            # Drawing ball path
            image = cv2.line(frame_markers, (int(x), int(y)), (int(x_robot_corner), int(y_pred)), (255, 200, 200), 1) 

      if xd < 0:
        xd_array = []
        yd_array = []

    ###########
    # Drawing #
    ###########
    # output_image = np.zeros((h_frame, w_frame, 3))
    output_image = frame_markers.copy()
    # for i in range(n_arucos):
    #     image = cv2.circle(frame_markers, arucosCenter[i], radius=4, color=(255, 0, 255), thickness=-1)
    
    output_image = cv2.circle(output_image, (int(x), int(y)), radius=int(radius), color=(200, 255, 200), thickness=-1)
    image = cv2.arrowedLine(output_image, (int(x), int(y)), (int(x+10*xd_pred), int(y+10*yd_pred)), (255, 0, 0), 2) 
    if top_left_corner.any() or bot_right_corner.any():
      output_image = cv2.circle(output_image, top_left_corner, radius=4, color=(0, 255, 0), thickness=-1)
      output_image = cv2.circle(output_image, bot_right_corner, radius=4, color=(0, 0, 255), thickness=-1)
      output_image = cv2.circle(output_image, (40, 20), radius=4, color=(0, 0, 255), thickness=-1)
      output_image = cv2.rectangle(output_image, top_left_corner, bot_right_corner, color=(200, 200, 200), thickness=1)
    # cv2.putText(output_image, "Robotic pong", (int(w_frame/2) - 70, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    
    # Moving the robot
    output_image = cv2.rectangle(output_image, (int(x_robot_corner - 4), int(y_pred - 20)), (int(x_robot_corner + 4), int(y_pred + 20)), color=(255, 255, 255), thickness=-1)
    
    cv2.imshow('Processed',output_image)
    
    # Press Q on keyboard to  exit
    if cv2.waitKey(25) & 0xFF == ord('q'):
      break
 
  # Break the loop
  else: 
    break
 
# When everything done, release the video capture object
cap.release()
 
# Closes all the frames
cv2.destroyAllWindows()