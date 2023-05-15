from pyniryo2 import *


robot = NiryoRobot("169.254.200.200")

robot.arm.calibrate_auto()
robot.arm.set_arm_max_velocity(100)

while True:
    try:
        value = int(input('robot_position: '))
        robot_pos = -0.5*value/100 + 0.25
        print("Moving to: ", robot_pos)
        robot.arm.stop_move()
        robot.arm.stop_move()
        robot.arm.move_linear_pose([0.3, robot_pos, 0.1, 0.0, 1.57, 0.0], callback=lambda _: print(""))
    except:
        break

robot.end()