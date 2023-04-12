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


def is_point_inside_frame(point, h, w):
    if (0 <= point[0] <= w) and (0 <= point[1] <= h):
        return True
    return False

def detectBall(frame):
    frame = frame.copy()
    blurred = cv2.GaussianBlur(frame, (11, 11), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

    # construct a mask for the color "green", then perform
    # a series of dilations and erosions to remove any small
    # blobs left in the mask

    mask = cv2.inRange(hsv, greenLower, greenUpper)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)
    frame_cnts, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    if len(frame_cnts) > 0:
        c = max(frame_cnts, key=cv2.contourArea)
        ((x, y), radius) = cv2.minEnclosingCircle(c)
        diameter = 2*radius
        M = cv2.moments(c)
        if diameter > 10:
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
            cv2.circle(frame, (int(x), int(y)), int(radius), (0, 0, 255), 2)
            return True, frame, frame_cnts, x, y, radius
    print("Ball not detected")
    return False, frame, [], 0, 0, 0

# def predict_ball_target():


def delimit_field(frame, n_arucos):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
    parameters =  cv2.aruco.DetectorParameters_create()
    corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
    frame_markers = cv2.aruco.drawDetectedMarkers(frame.copy(), corners, ids)
    corners = np.array(corners, dtype=int)
    if len(corners) == n_arucos:
        # The top left corner is identified by the Aruco with ID = 0
        top_left_corner_id = np.where(ids == 0)[0]
        # The top left corner is identified by the Aruco with ID = 0
        bottom_right_corner_id = np.where(ids == 1)[0]
        top_left_corner = corners[top_left_corner_id][0][0][0]
        bottom_right_corner = corners[bottom_right_corner_id][0][0][0]

        return True, frame, [top_left_corner, bottom_right_corner]
    return False, frame_markers, [[], []]