import serial 
import time

class SerialStepperMotor:
    def __init__(self, port, baudrate):
        self.port = port
        self.baudrate = baudrate
        self.serial_connection = serial.Serial(self.port, self.baudrate, timeout=1)
        time.sleep(1)

    def is_connected(self):
        self.serial_connection.write(bytearray("c 200", 'ascii'))
        for _ in range(10):
            answer = self.serial_connection.readline()
            if answer[0:3] == b'200':
                return True
            time.sleep(0.5)
        else:
            return False
    
    def calibrate(self):
        print("Starting calibration...")
        self.serial_connection.write(bytearray("i", 'ascii'))
        calibration_finished = False
        while not calibration_finished:
            answer = self.serial_connection.readline()
            if answer[0:3] == b'201':
                calibration_finished = True
        print("Calibrated...")

    def close(self):
        self.serial_connection.close()

    def move_to(self, position_percentage):
        """
        Receives position in percentage and moves the motor 
        to the equivalent percentage of its full trajectory.
        """
        self.serial_connection.write(bytearray("m " + str(position_percentage) +" \n", 'ascii'))
        print("Sent: m " + str(position_percentage))

    def stop(self):
        self.serial_connection.write(bytearray("p \n", 'ascii'))
        print("Sent: p")

# # Use example
# motor = SerialStepperMotor("COM8", 115200, 0, 2700)
# print(motor.is_connected())
# motor.calibrate()
# time.sleep(10)
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