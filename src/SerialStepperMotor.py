import serial 
import time

class SerialStepperMotor:
    def __init__(self, port, baudrate, pos_min_motor, pos_max_motor):
        self.port = port
        self.baudrate = baudrate
        self.serial_connection = serial.Serial(self.port, self.baudrate, timeout=1)
        self.pos_min_motor = pos_min_motor
        self.pos_max_motor = pos_max_motor
        time.sleep(3)

    def is_connected(self):
        self.serial_connection.write(bytearray("c 200", 'ascii'))
        time.sleep(1)
        answer = self.serial_connection.readline()
        if answer[0:3] == b'200':
            return True
        else:
            return False
    
    def close(self):
        self.serial_connection.close()

    def move_to(self, position_percentage):
        """
        Receives position in percentage and moves the motor 
        to the equivalent percentage of its full trajectory.
        """
        steps = int(position_percentage*(self.pos_max_motor - self.pos_min_motor) + self.pos_min_motor)
        self.serial_connection.write(bytearray("m " + str(steps) +" \n", 'ascii'))
        
    def stop(self):
        self.serial_connection.write(bytearray("p \n", 'ascii'))

# # Use example
# motor = SerialStepperMotor("COM8", 115200, 0, 2700)
# print(motor.is_connected())
# while True:
#     try:
#         time.sleep(2)
#         motor.move_to(0)
#         time.sleep(2)
#         motor.move_to(1)
#     except:
#         break
# motor.stop()
# motor.close()