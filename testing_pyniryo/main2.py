from threading import Thread
from pyniryo import *
import time
robot = NiryoRobot("169.254.200.200")

robot.calibrate_auto()


# create the thread
thread = Thread(target=robot.move_linear_pose, args=([0.3, 0.25, 0.1, 0.0, 1.57, 0.0]))
# report the thread is alive
print(thread.is_alive())
# start the thread
thread.start()
# report the thread is alive

print(thread.is_alive())
time.sleep(10)

print(thread.is_alive())
# wait for the thread to finish
thread.join()
# report the thread is alive
print(thread.is_alive())


# create the thread
thread = Thread(target=robot.move_linear_pose, args=([0.3, -0.25, 0.1, 0.0, 1.57, 0.0]))
# report the thread is alive
print(thread.is_alive())
# start the thread
thread.start()
# report the thread is alive
print(thread.is_alive())
# wait for the thread to finish
thread.join()
# report the thread is alive
print(thread.is_alive())

robot.close_connection()