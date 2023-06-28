from motor_configs import *
import time
from threading import Thread

motor = MotorEPOS()

print(motor.get_position())

while True:
    # try:
    value = float(input('robot_position: '))
    thread = Thread(target=motor.move_to_position, args=([value]))
    thread.start()
    time.sleep(0.5)
    motor.stop_motor_movement()
    thread.join()
    
    # except:
    #     break

motor.close()