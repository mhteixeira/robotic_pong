import cv2
from helpers import *
from params import *
from threading import Thread
import numpy as np
from pyniryo2 import *

h, w = 512, 1024
cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)

hit_region_width = 60

hit_angle = 45

movement_finished = False

def update_hit_angle():
    global hit_angle
    global movement_finished
    try:
        print(movement_finished)
        hit_angle = int(input("Angle (deg) to hit the ball [-45, 45]: "))
        movement_finished = False
    except:
        print("Invalid angle")

thread = Thread(target = update_hit_angle)

top_left_corner = np.array([])
bottom_right_corner = np.array([])


is_field_delimited = False
while ((not is_field_delimited) and cap.isOpened()):
    ret, frame = cap.read()
    if ret == True:
        frame = resize_image(frame, 60)
        is_field_delimited, output_frame, corners = delimit_field(frame)
        if (is_field_delimited):
            top_left_corner = corners[0]
            bottom_right_corner = corners[1]
            field_height = bottom_right_corner[1] - top_left_corner[1]
            x_robot_corner = bottom_right_corner[0]
            output_frame = cv2.rectangle(output_frame, tuple(top_left_corner), tuple(bottom_right_corner), color=field_limits_rect_color, thickness=2)
            arm_corner = corners[2]
        cv2.imshow('frame',output_frame)

        # Press Q on keyboard to  exit
        if cv2.waitKey(25) & 0xFF == ord('q'):
            break
    else: 
        break

pixel_density = 0
robot = NiryoRobot("169.254.200.200")

field_lenght_m = 0.64 - 0.08

def move_callback(_):
    print("Move completed")
        
robot.arm.calibrate_auto()

while True:
    ret, original_frame = cap.read()
    if ret == True:

        is_field_delimited, output_frame, corners = delimit_field(original_frame)
        if (is_field_delimited):
            top_left_corner = corners[0]
            bottom_right_corner = corners[1]
            field_height = bottom_right_corner[1] - top_left_corner[1]
            x_robot_corner = bottom_right_corner[0]
            pixel_density = field_height/field_lenght_m

            arm_corner = corners[2]
        output_frame = cv2.circle(output_frame, arm_corner, radius=2, color=(0, 255, 255), thickness=2)

        output_frame = cv2.rectangle(output_frame, tuple(top_left_corner), tuple(bottom_right_corner), color=field_limits_rect_color, thickness=2)

        is_ball_detected, _, _, x, y, radius = detect_ball(original_frame, output_frame)

        output_frame = cv2.line(output_frame, (top_left_corner[0] + hit_region_width, bottom_right_corner[1]), (top_left_corner[0] + hit_region_width, top_left_corner[1]), color=(0, 255, 255), thickness=1)
        if not movement_finished:
            print("oi")
            slope = np.tan(np.deg2rad(hit_angle))
            initial_position = slope*(top_left_corner[0] - int(x)) + int(y)

            initial_pose = np.array([top_left_corner[0] - arm_corner[0], arm_corner[0] - initial_position])/pixel_density
            initial_pose[0] += 0.10
            robot.arm.move_linear_pose([initial_pose[0], initial_pose[1], 0.09, 0.0, 1.57, 0.0])
            
            final_position = slope*(top_left_corner[0] + hit_region_width - int(x)) + int(y)
            
            final_pose = np.array([top_left_corner[0]  + hit_region_width - arm_corner[0], arm_corner[0] - final_position])/pixel_density
            final_pose[0] += 0.10
            
            robot.arm.move_linear_pose([final_pose[0], final_pose[1], 0.09, 0.0, 1.57, 0.0])
            movement_finished = True
        
        output_frame = cv2.circle(output_frame, (top_left_corner[0], int(initial_position)), radius=2, color=(0, 0, 255), thickness=2)
        output_frame = cv2.circle(output_frame, (top_left_corner[0] + hit_region_width, int(final_position)), radius=2, color=(0, 0, 255), thickness=2)
        cv2.imshow('frame', output_frame)
        
        if not thread.is_alive():
            thread = Thread(target=update_hit_angle)
            thread.start()

    if cv2.waitKey(25) & 0xFF == ord('q'):
        break

robot.end()