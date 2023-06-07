import cv2
import numpy as np
from params import *

def resize_image(img, scale_percent):
    width = int(img.shape[1] * scale_percent / 100)
    height = int(img.shape[0] * scale_percent / 100)
    dim = (width, height)
    
    # resize image
    resized = cv2.resize(img, dim, interpolation = cv2.INTER_AREA)
    return(resized)

def line_limits(corners, frame):
    m, n = np.polyfit(corners[:, 0], corners[:, 1], 1)
    h, w, _ = frame.shape
    possible_limits = np.array([
        (0, int(n)),            # f(0) = y
        (w, int(w*m + n)),      # f(w) = y
        (int(-n/m), 0),         # f(x) = 0 = m*x + n => x = -n/m
        (int((h - n)/m), h)     # f(x) = h = m*x + n => x = (h - n)/m
    ])
    points_inside_frame = [
        True if is_point_inside_frame(point, h, w) else False 
        for point in possible_limits]
    limits = possible_limits[points_inside_frame]
    return limits

def is_ball_inside_field(x, y, x_min, y_min, x_max, y_max):
    if (x_min <= x <= x_max) and (y_min <= y <= y_max):
        return True
    return False

def detect_ball(frame, output_frame):
    # First we blur the image with a GaussianBlur
    blurred = cv2.GaussianBlur(frame, (11, 11), 0)
    
    # Construct a HSV mask for the green color
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, greenLower, greenUpper)

    # Erode and dilate the result to remove small noises
    mask = cv2.erode(mask, None, iterations=4)
    mask = cv2.dilate(mask, None, iterations=4)
    
    # Then we calculate the countours of the resulting image
    frame_cnts, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(output_frame, frame_cnts, -1, (0, 0, 255), 2)
    
    if len(frame_cnts) > 0:
        # If we have contours, we get the one with the greatest area
        c = max(frame_cnts, key=cv2.contourArea)
        
        # And calculate the minimum enclosing circle
        ((x, y), radius) = cv2.minEnclosingCircle(c)
        M = cv2.moments(c)

        # Calculate the shape
        approx = cv2.approxPolyDP(c,0.01*cv2.arcLength(c,True),True)
        # If the shape is close to a circle and the area is greater than the minimum
        # the contour is considered to be the ball
        if ((len(approx) > 8) & (len(approx) < 23) & (M["m00"] > minimum_ball_area)):
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
            cv2.circle(output_frame, (int(x), int(y)), int(radius), (0, 255, 0), 2)

            return True, output_frame, frame_cnts, x, y, radius
    return False, output_frame, [], 0, 0, 0

def predict_ball_target(frame, kf, ball_position, xd_array, yd_array, x_robot_corner, top_left_corner, bottom_right_corner, y_preds, is_going_to_bounce, current_pred):
    # Kalman predictions
    kf.correct(np.array(ball_position, dtype=np.float32))
    _, _, xd_pred, yd_pred = kf.predict()
    y_pred = current_pred
    field_height = (bottom_right_corner[1] - top_left_corner[1])
    if xd_pred and yd_pred:
        xd_array.append(xd_pred)
        yd_array.append(yd_pred)

    if len(xd_array) > initializing_window + 1:

        if len(xd_array) > moving_average_window_size + initializing_window:
            xd = np.mean(xd_array[-moving_average_window_size:])
            yd = np.mean(yd_array[-moving_average_window_size:])
        else:
            xd = np.mean(xd_array[initializing_window:-1])
            yd = np.mean(yd_array[initializing_window:-1])
            
        slope = round(yd/xd, 2)
        if xd > 0 and (xd > 0.3 or abs(yd) > 0.3):
            initial_y_pred = slope*(x_robot_corner-ball_position[0]) + ball_position[1]
            is_going_to_bounce = not (bottom_right_corner[1] > initial_y_pred > top_left_corner[1])
            if initial_y_pred < top_left_corner[1]:
                overshoot_y = abs(top_left_corner[1] - initial_y_pred)
                number_of_refletions = np.ceil(overshoot_y / field_height).astype(int)
                if number_of_refletions <= max_number_of_refletions:
                    if (number_of_refletions % 2 == 1):
                        y_pred = top_left_corner[1] + (overshoot_y % field_height)
                    if (number_of_refletions % 2 == 0):
                        y_pred = bottom_right_corner[1] - overshoot_y % field_height
                    current_x = ball_position[0]
                    next_x = ball_position[0]
                    current_y = ball_position[1]
                    next_y = ball_position[1]
                    for n in range(1, number_of_refletions + 1):
                        if n % 2 == 1:
                            next_x = (top_left_corner[1] - current_y)/slope + current_x
                            next_y = top_left_corner[1]
                            cv2.line(frame, (int(current_x), int(current_y)), (int(next_x), top_left_corner[1]), predict_line_color, 1)     
                        if n % 2 == 0:
                            next_x = -(bottom_right_corner[1] - current_y)/slope + current_x
                            next_y = bottom_right_corner[1]
                            image = cv2.line(frame, (int(current_x), int(current_y)), (int(next_x), bottom_right_corner[1]), predict_line_color, 1)     
                        current_x = next_x
                        current_y = next_y
                    image = cv2.line(frame, (int(current_x), int(current_y)), (int(x_robot_corner), int(y_pred)), predict_line_color, 1)     
            elif initial_y_pred > bottom_right_corner[1]:

                overshoot_y = abs(initial_y_pred - bottom_right_corner[1])
                number_of_refletions = np.ceil(abs(overshoot_y / field_height)).astype(int)
                
                if number_of_refletions <= max_number_of_refletions:
                    if (number_of_refletions % 2 == 1):
                        y_pred = bottom_right_corner[1] - (overshoot_y % field_height)
                    if (number_of_refletions % 2 == 0):
                        y_pred = overshoot_y % field_height + top_left_corner[1]
                    current_x = ball_position[0]
                    next_x = ball_position[0]
                    current_y = ball_position[1]
                    next_y = ball_position[1]
                    for n in range(1, number_of_refletions + 1):
                        if n % 2 == 1:
                            next_x = (bottom_right_corner[1] - current_y)/slope + current_x
                            next_y = bottom_right_corner[1]
                            cv2.line(frame, (int(current_x), int(current_y)), (int(next_x), bottom_right_corner[1]), predict_line_color, 1)     
                        if n % 2 == 0:
                            next_x = -(top_left_corner[1] - current_y)/slope + current_x
                            next_y = top_left_corner[1]
                            image = cv2.line(frame, (int(current_x), int(current_y)), (int(next_x), top_left_corner[1]), predict_line_color, 1)     
                        current_x = next_x
                        current_y = next_y
                    image = cv2.line(frame, (int(current_x), int(current_y)), (int(x_robot_corner), int(y_pred)), predict_line_color, 1)     
            else:
                y_pred = initial_y_pred
                image = cv2.line(frame, (int(ball_position[0]), int(ball_position[1])), (int(x_robot_corner), int(y_pred)), predict_line_color, 1) 
        if xd < 0:
            xd_array = []
            yd_array = []
            y_preds = []
    
    return frame, y_pred, xd_pred, yd_pred, is_going_to_bounce

def delimit_field(frame):
    # First we make the image monochromatic
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Next we configure the ArUco detector
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
    parameters =  cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)
    
    # Then we detect and draw the ArUcos
    corners, ids, rejectedImgPoints = detector.detectMarkers(gray)
    frame_markers = cv2.aruco.drawDetectedMarkers(frame.copy(), corners, ids)
    corners = np.array(corners, dtype=int)
    
    # We now check if the amount of detected ArUcos is the expected one
    # If it is not, the detection failed
    if len(corners) == n_arucos:
        
        # The top left corner is identified by the Aruco with ID = 0
        top_left_corner_id = np.where(ids == 0)[0]
        
        # The bottom right corner is identified by the Aruco with ID = 1
        bottom_right_corner_id = np.where(ids == 1)[0]

        # The arm position is identified by the Aruco with ID = 124
        arm_corner_id = np.where(ids == 124)[0]
        
        # We select the aruco from the 
        top_left_corner = corners[top_left_corner_id][0][0][0]
        bottom_right_corner = corners[bottom_right_corner_id][0][0][0]
        arm_corner = corners[arm_corner_id][0][0][0]

        return True, frame_markers, [top_left_corner, bottom_right_corner, arm_corner]
    return False, frame_markers, [[], []]

def y_robot_to_robot_position(y_robot, top_left_corner, bottom_right_corner):
    return 1.2*(y_robot - top_left_corner[1])/(bottom_right_corner[1] - top_left_corner[1]) - 0.6


def non_blocking_move_linear_position(robot, pose):
    robot.arm._actions.get_move_linear_pose_goal(pose).send()