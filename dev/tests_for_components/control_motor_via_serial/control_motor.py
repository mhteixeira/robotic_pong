import serial
import time

ser = serial.Serial("COM8", 115200, timeout=1)
time.sleep(3)
ser.write(bytearray("c 200", 'ascii'))
time.sleep(1)
answer = ser.readline()

while True:
    command = input()
    ser.write(bytearray("m "+command+"\n", 'ascii'))
#     time.sleep(1)
#     ser.write(bytearray("m 400\n", 'ascii'))
#     time.sleep(1)

ser.close()