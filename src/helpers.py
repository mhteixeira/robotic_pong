import cv2
import numpy as np
from params import *

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
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)
    
    # Then we calculate the countours of the resulting image
    frame_cnts, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    # cv2.drawContours(output_frame, frame_cnts, -1, (0,0,255), 2)

    if len(frame_cnts) == 0:
        return False, output_frame, [], -1, -1, -1

    # Calculate the circularity of the identified countours
    areas = np.array([cv2.contourArea(c) for c in frame_cnts])
    is_reading_valid = (areas > minimum_ball_area) & (areas < maximum_ball_area)

    if np.sum(is_reading_valid) == 0:
        return False, output_frame, [], -1, -1, -1

    perimeters = np.array([cv2.arcLength(c,True) for c in frame_cnts])

    circularities = 4 * np.pi * areas/(perimeters**2)
    circularities = circularities*is_reading_valid
    ball_cnt_idx = np.argmax(circularities)
    # We get the one with the greatest circularity (4*pi*area/(perimeter^2))
    # https://www.mathworks.com/help/images/identifying-round-objects.html
    c = frame_cnts[ball_cnt_idx]
    # And calculate the minimum enclosing circle
    ((x, y), radius) = cv2.minEnclosingCircle(c)
    M = cv2.moments(c)
    
    # Calculate the shape
    approx = cv2.approxPolyDP(c,0.01*cv2.arcLength(c,True),True)

    # If the shape is really close to a circle and the area is greater than the minimum
    # the contour is considered to be the ball
    if (len(approx) > 5) & (len(approx) < 23):
        center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
        cv2.circle(output_frame, (int(x), int(y)), int(radius), (0, 255, 0), 2)

        return True, output_frame, frame_cnts, x, y, radius
    return False, output_frame, [], -1, -1, -1

def non_blocking_move_linear_position(robot, pose, callback=None):
    robot.arm._actions.get_move_linear_pose_goal(pose).send(result_callback=callback)

def warp_point(x: int, y: int, M):
	d = M[2, 0] * x + M[2, 1] * y + M[2, 2]

	return (
		int((M[0, 0] * x + M[0, 1] * y + M[0, 2]) / d), # x
		int((M[1, 0] * x + M[1, 1] * y + M[1, 2]) / d), # y
	)

def predict_target(frame, kf, ball_position, xd_array, yd_array, x_robot_corner, max_width, max_height, y_preds, is_going_to_bounce, current_pred):
    top_left_corner = np.array([0, 0])
    bottom_right_corner = np.array([max_width, max_height])
    
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
        if xd > 0 and (xd > 0.2 or abs(yd) > 0.2):
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
                
                if number_of_reflemaxtions <= max_number_of_refletions:
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

def custom_set_arm_max_velocity(arm, percentage_speed):
        arm._check_range_belonging(percentage_speed, 1, 200)
        req = arm._services.get_max_velocity_scaling_factor_request(percentage_speed)
        resp = arm._services.set_max_velocity_scaling_factor_service.call(req)
        arm._check_result_status(resp)
