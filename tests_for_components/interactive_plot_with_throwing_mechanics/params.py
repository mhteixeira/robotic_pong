import numpy as np

# Calibração
field_lenght_m = 0.64 # m
aruco_0_pose = np.array([0.14, 0.28, 0.09, 0.0, 1.57, 0.0])
aruco_1_pose = np.array([0.14, -0.28, 0.09, 0.0, 1.57, 0.0])
hit_region = 80

# Ball parameters
ball_color_rgb = (169, 78, 60)
ball_color_hsv = (5, 165, 169)

greenLower = (0, 80, 0)
greenUpper = (20, 250, 200)

# greenLower = (20, 100, 6)
# greenUpper = (64, 220, 255)
minimum_ball_area = 100
maximum_ball_area = 1500

# Field parameters
n_arucos = 3
bounce_margin_size = 40

# Moving average parameters
moving_average_window_size = 30
initializing_window = 8

# Reflection parameters
max_number_of_refletions = 3

# Image colors
field_limits_rect_color = (255, 100, 100)
predict_line_color = (255, 150, 150)