from pyniryo2 import *
import time

robot = NiryoRobot("169.254.200.200")

field_lenght_m = 0.64 - 0.08

def move_callback(_):
    print("Move completed")
        
robot.arm.calibrate_auto()

robot.arm.move_linear_pose([0.25, 0.01, 0.09, 0.0, 1.57, 0.0], callback=lambda _: print("Move completed"))
print("Stopped?")

print("Ending...")
robot.end()
print("Ended")
