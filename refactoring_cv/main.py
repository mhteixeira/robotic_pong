import cv2
import numpy as np
import time
from helpers import resize_image, detectBall, is_point_inside_frame, line_limits, delimit_field
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D

cap = cv2.VideoCapture('refactoring_cv/examples/example53.avi')
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
top_left_corner = np.array([])
bottom_right_corner = np.array([])
lim_x = 0
lim_y = 0
x_robot_corner = 0
max_number_of_refletions = 3
moving_average_window_size = 10
initializing_window = 5
frame_count = 0

xd_array = []
yd_array = []
y_pred = h_frame/2

# Kalman filter
kf = cv2.KalmanFilter(4, 2)
kf.measurementMatrix = np.array([[1, 0, 0, 0], [0, 1, 0, 0]], np.float32)
kf.transitionMatrix = np.array([[1, 0, 1, 0], [0, 1, 0, 1], [0, 0, 1, 0], [0, 0, 0, 1]], np.float32)

# Matplotlib figure and axis
fig, ax = plt.subplots()

# Initialize the graph
x_plot = []
y1 = []
y2 = []
ax.set_xlabel('X-axis')
ax.set_ylabel('Y-axis')
ax.set_title('Unfiltered vs filtered predictions')

# First make sure that the field is delimited
is_field_delimited = False
while (not is_field_delimited):
  ret, frame = cap.read()
  if ret == True:
    frame = resize_image(frame, 100)
    is_field_delimited, frame_markers, corners = delimit_field(frame, n_arucos)
    if (is_field_delimited):
      top_left_corner = corners[0]
      bottom_right_corner = corners[1]
      field_height = bottom_right_corner[1] - top_left_corner[1]
      x_robot_corner = bottom_right_corner[0]
    cv2.imshow('Original',frame_markers)
    # Press Q on keyboard to  exit
    if cv2.waitKey(25) & 0xFF == ord('q'):
      break
  else: 
    break

# Start the processing
while(cap.isOpened()):

  ret, frame = cap.read()
  if ret == True:
    frame = resize_image(frame, 100)
    h_frame, w_frame, _ = frame.shape

    # Delimit field again to make the system robust to 
    # changes in position of the arucos and the camera
    is_field_delimited, frame_markers, corners = delimit_field(frame, n_arucos)
    if (is_field_delimited):
      top_left_corner = corners[0]
      bottom_right_corner = corners[1]
      field_height = bottom_right_corner[1] - top_left_corner[1]
      x_robot_corner = bottom_right_corner[0]

    # Detect the ball
    is_ball_detected, _, _, x, y, radius = detectBall(frame)
    
    # The prediction only makes sense if the ball is present
    if (True):
      # Kalman predictions
      kf.correct(np.array([x, y], dtype=np.float32))
      _, _, xd_pred, yd_pred = kf.predict()
      

      if xd_pred and yd_pred:
        xd_array.append(xd_pred)
        yd_array.append(yd_pred)
      
      frame_count += 1
      x_plot.append(frame_count)
      y1.append(yd_pred/xd_pred if xd_pred != 0 else 0)


      if len(xd_array) > initializing_window + 1:

        if len(xd_array) > moving_average_window_size + initializing_window:
          xd = np.mean(xd_array[-moving_average_window_size:])
          yd = np.mean(yd_array[-moving_average_window_size:])
        else:
          xd = np.mean(xd_array[initializing_window:-1])
          yd = np.mean(yd_array[initializing_window:-1])
          
        slope = round(yd/xd, 2)
        y2.append(slope)
        if xd > 0 and (xd > 0.3 or abs(yd) > 0.3):
          initial_y_pred = slope*(x_robot_corner-x) + y
          if initial_y_pred < top_left_corner[1]:
            overshoot_y = abs(top_left_corner[1] - initial_y_pred)
            number_of_refletions = np.ceil(overshoot_y / field_height).astype(int)
            if number_of_refletions <= max_number_of_refletions:
              if (number_of_refletions % 2 == 1):
                y_pred = top_left_corner[1] + (overshoot_y % field_height)
              if (number_of_refletions % 2 == 0):
                y_pred = overshoot_y % field_height
              current_x = x
              next_x = x
              current_y = y
              next_y = y
              for n in range(1, number_of_refletions + 1):
                if n % 2 == 1:
                  next_x = (top_left_corner[1] - current_y)/slope + current_x
                  next_y = top_left_corner[1]
                  cv2.line(frame_markers, (int(current_x), int(current_y)), (int(next_x), top_left_corner[1]), (255, 200, 200), 1)     
                if n % 2 == 0:
                  next_x = -(bottom_right_corner[1] - current_y)/slope + current_x
                  next_y = bottom_right_corner[1]
                  image = cv2.line(frame_markers, (int(current_x), int(current_y)), (int(next_x), bottom_right_corner[1]), (255, 200, 200), 1)     
                current_x = next_x
                current_y = next_y
              image = cv2.line(frame_markers, (int(current_x), int(current_y)), (int(x_robot_corner), int(y_pred)), (255, 200, 200), 1)     
          elif initial_y_pred > bottom_right_corner[1]:
            overshoot_y = abs(initial_y_pred - bottom_right_corner[1])
            number_of_refletions = np.ceil(abs(overshoot_y / field_height)).astype(int)
            if number_of_refletions <= max_number_of_refletions:
              if (number_of_refletions % 2 == 1):
                y_pred = bottom_right_corner[1] - (overshoot_y % field_height)
              if (number_of_refletions % 2 == 0):
                y_pred = overshoot_y % field_height
              current_x = x
              next_x = x
              current_y = y
              next_y = y
              for n in range(1, number_of_refletions + 1):
                if n % 2 == 1:
                  next_x = (bottom_right_corner[1] - current_y)/slope + current_x
                  next_y = bottom_right_corner[1]
                  cv2.line(frame_markers, (int(current_x), int(current_y)), (int(next_x), bottom_right_corner[1]), (255, 200, 200), 1)     
                if n % 2 == 0:
                  next_x = -(top_left_corner[1] - current_y)/slope + current_x
                  next_y = top_left_corner[1]
                  image = cv2.line(frame_markers, (int(current_x), int(current_y)), (int(next_x), top_left_corner[1]), (255, 200, 200), 1)     
                current_x = next_x
                current_y = next_y
              image = cv2.line(frame_markers, (int(current_x), int(current_y)), (int(x_robot_corner), int(y_pred)), (255, 200, 200), 1)     
          else:
            y_pred = initial_y_pred
            image = cv2.line(frame_markers, (int(x), int(y)), (int(x_robot_corner), int(y_pred)), (255, 200, 200), 1) 
        if xd < 0:
          xd_array = []
          yd_array = []
      else:
        y2.append(0)

      ###########
      # Drawing #
      ###########
      # output_image = np.zeros((h_frame, w_frame, 3))
      output_image = frame_markers
      # for i in range(n_arucos):
      #     image = cv2.circle(frame_markers, arucosCenter[i], radius=4, color=(255, 0, 255), thickness=-1)
      
      output_image = cv2.circle(output_image, (int(x), int(y)), radius=int(radius), color=(100, 255, 100), thickness=2)
      image = cv2.arrowedLine(output_image, (int(x), int(y)), (int(x+10*xd_pred), int(y+10*yd_pred)), (255, 0, 0), 2) 
      if top_left_corner.any() or bottom_right_corner.any():
        output_image = cv2.circle(output_image, top_left_corner, radius=4, color=(0, 255, 0), thickness=-1)
        output_image = cv2.circle(output_image, bottom_right_corner, radius=4, color=(0, 0, 255), thickness=-1)
        output_image = cv2.circle(output_image, (40, 20), radius=4, color=(0, 0, 255), thickness=-1)
        output_image = cv2.rectangle(output_image, top_left_corner, bottom_right_corner, color=(200, 200, 200), thickness=1)
      # cv2.putText(output_image, "Robotic pong", (int(w_frame/2) - 70, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
      
      # Moving the robot
      output_image = cv2.rectangle(output_image, (int(x_robot_corner - 6), int(y_pred - 30)), (int(x_robot_corner + 6), int(y_pred + 30)), color=(255, 255, 255), thickness=-1)
      
    cv2.imshow('Processed',frame_markers)
    
    # Press Q on keyboard to  exit
    if cv2.waitKey(25) & 0xFF == ord('q'):
      break
 
  # Break the loop
  else: 
    break
 

# line1.set_data(x_plot, y1)
# line2.set_data(x_plot, y2)
plt.plot(x_plot, y1, x_plot, y2)
plt.legend(['Unfiltered', 'Filtered'])
plt.ylim([-1, 5])
plt.show()


# When everything done, release the video capture object
cap.release()
 
# Closes all the frames
cv2.destroyAllWindows()