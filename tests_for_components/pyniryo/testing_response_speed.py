from pyniryo2 import *
import time

robot = NiryoRobot("169.254.200.200")

robot.arm.calibrate_auto()
robot.arm.set_arm_max_velocity(100)

# Subscribe a callback
def velocity_callback(velocity_percentage):
    print('vel', velocity_percentage)

while True:
    try:
        value = int(input('robot_position: '))
        robot_pos = -1.2*value/100 + 0.6
        print("Moving to: ", robot_pos)
        robot.arm.stop_move()
        time.sleep(0.01)
        robot.arm.move_joints([robot_pos, -1.552, 1.29, 0.0, -1.2, 0.0])
        # robot.arm.move_pose([0.3, robot_pos, 0.1, 0.0, 1.57, 0.0], callback=lambda x: print(""))
        # pose_list = [0.3, robot_pos, 0.1, 0.0, 1.57, 0.0]
        # goal = robot.arm._actions.get_move_linear_pose_goal(pose_list)
        # goal.send()
    except:
        break

robot.end()