from random import random 
from time import sleep
from threading import Thread
import sys


class Motor:
    def __init__(self):
        self.flag = 1
    
    def set_flag(self, value):
        self.flag = value
    
    def task(self, time):
        print("Starting to move the motor")
        elapsed_time = 0

        while elapsed_time < time:
            sleep(0.1)
            elapsed_time += 0.1
            
            print("Moving the motor, flag =", self.flag)
            if self.flag == 2:
                print('Closing thread')
                return
        print("Finished")

motor = Motor()

# create and configure the new thread
thread = Thread(target=motor.task, args=([3]))
# start the new thread
thread.start()

sleep(2)
motor.set_flag(2)

# wait for the thread to terminate
thread.join()
# main continues on
print('Main continuing on...')