import serial
import time
from ctypes import *
import sys

# EPOS Command Library path
path='./tests_for_components/linear_actuator/EposCmd64.dll'

# Load library
cdll.LoadLibrary(path)
epos = CDLL(path)

# Defining return variables from Library Functions
ret = 0
pErrorCode = c_uint()
pDeviceErrorCode = c_uint()

# Defining a variable NodeID and configuring connection for device 1
nodeID = 1
baudrate = 115200
timeout = 500

EC_PID_POSITION_CONTROLLER = 20
EC_PIDPC_P_GAIN = 1
EC_PIDPC_I_GAIN = 2
EC_PIDPC_D_GAIN = 3

EC_PI_VELOCITY_CONTROLLER = 10
EG_PIVC_P_GAIN = 1
EG_PIVC_I_GAIN = 2

Kp = 7000
Ki = 500

upper_position_limit = -15148
lower_position_limit = 38755
max_velocity = 1600
rate = 3000 # dx/s @ 1600 rpm

class MotorEPOS:
    # Initiating connection and setting motion profile for EPOS
    def __init__(self):
        self.stop_motor_movement_flag = False
        self.is_currently_moving = False

        self.keyHandle = epos.VCS_OpenDevice(b'EPOS', b'MAXON_RS232', b'RS232', b'COM22', byref(pErrorCode)) # specify EPOS version and interface
        if self.keyHandle == 0:
            print('Motor not connected')
            sys.exit()

        if epos.VCS_SetProtocolStackSettings(self.keyHandle, baudrate, timeout, byref(pErrorCode)) == 0:
            print('EPOS protocol not set')
            sys.exit()

        if epos.VCS_ClearFault(self.keyHandle, nodeID, byref(pErrorCode)) == 0:
            print('faults not cleared')

        if epos.VCS_ActivateVelocityMode(self.keyHandle, nodeID, byref(pErrorCode)) == 0:
            print('Velocity mode not activated')

        if epos.VCS_SetEnableState(self.keyHandle, nodeID, byref(pErrorCode)) == 0:
            print('state not enabled')

        if epos.VCS_SetControllerGain(self.keyHandle, nodeID, EC_PI_VELOCITY_CONTROLLER, EG_PIVC_P_GAIN, Kp, byref(pErrorCode)) == 0:
            print('P gain not set')

        if epos.VCS_SetControllerGain(self.keyHandle, nodeID, EC_PI_VELOCITY_CONTROLLER, EG_PIVC_I_GAIN, Ki, byref(pErrorCode)) == 0:
            print('I gain not set')

        time.sleep(1)
        print('EPOS initialized\n')

    def close(self):
        if epos.VCS_HaltVelocityMovement(self.keyHandle, nodeID, byref(pErrorCode)) == 0:
            print('couldnt halt velocity 1')    
        
        if epos.VCS_SetDisableState(self.keyHandle, nodeID, byref(pErrorCode)) == 0: # disable device 1
            print('device not disabled')
        
        print('device disabled\n')
            
        epos.VCS_CloseDevice(self.keyHandle, byref(pErrorCode)) # close device 1
        print('device closed\n')

    # Query motor position
    def get_position(self):
        pPositionIs=c_long()
        pErrorCode=c_uint()
        ret=epos.VCS_GetPositionIs(self.keyHandle, nodeID, byref(pPositionIs), byref(pErrorCode))
            
        return pPositionIs.value # motor steps

    def stop_motor_movement(self):
        print("updating flag, ", self.is_currently_moving)
        if self.is_currently_moving:
            self.stop_motor_movement_flag = True

    def set_velocity(self, rpm, duration):
        ret = epos.VCS_SetVelocityMust(self.keyHandle, nodeID, rpm, byref(pErrorCode))
        if ret == 0:
            print("velocity not set")
        else:
            self.is_currently_moving = True
            elapsed_time = 0
            while elapsed_time < duration:
                time.sleep(0.1)
                elapsed_time += 0.1
                print(f'flag: {self.stop_motor_movement_flag}', round(elapsed_time, 2), '/', duration)
                if self.stop_motor_movement_flag == True:
                    ret = epos.VCS_SetVelocityMust(self.keyHandle, nodeID, 0, byref(pErrorCode))
                    self.is_currently_moving = False
                    self.stop_motor_movement_flag = False
                    return
            self.is_currently_moving = False
            ret = epos.VCS_SetVelocityMust(self.keyHandle, nodeID, 0, byref(pErrorCode))

    def move_to_position(self, relative_position):
        # relative_position is a float between 0 and 1 
        # that tells how far along the trajectory the 
        # linear actuator should be

        final_position = relative_position*(upper_position_limit - lower_position_limit) + lower_position_limit
        current_position = self.get_position()
        duration = abs(final_position - current_position)/rate
        direction = -1 if final_position < current_position else 1
        print(final_position, current_position, duration, direction)
        self.set_velocity(direction*max_velocity, duration)