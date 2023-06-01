from pyniryo2 import *

robot = NiryoRobot("169.254.200.200")

def move_callback(_):
    print("Move completed")
        
robot.arm.calibrate_auto()

robot.arm.move_linear_pose([0.3, 0.25, 0.1, 0.0, 1.57, 0.0])
robot.arm.move_linear_pose([0.3, -0.25, 0.1, 0.0, 1.57, 0.0], callback=lambda _: print("Move completed"))
# robot.arm.stop_move()
# robot.arm.stop_move()
print("Stopped?")

print("Waiting...")
robot.wait(10)
print("Ending...")
robot.end()
print("Ended")

# In my terminal, after executing I get:

# Stopped?
# Move completed

# And the robot finishes the movement...

# Another problem is that, if I change the "move_pose" method to 
# the "move_linear_pose", which is actually the one I would like 
# to use. The output is:

# Move completed
# Stopped?

# Which indicated that the function, even with the callback, continues
# blocking the execution.