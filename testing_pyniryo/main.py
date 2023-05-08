from pyniryo import *

robot = NiryoRobot("169.254.200.200")

robot.calibrate_auto()
print("oi")
robot.move_linear_pose(0.3,  0.25, 0.1, 0.0, 1.57, 0.0)
robot.move_linear_pose(0.3, -0.25, 0.1, 0.0, 1.57, 0.0)
print("oi4")
robot.close_connection()