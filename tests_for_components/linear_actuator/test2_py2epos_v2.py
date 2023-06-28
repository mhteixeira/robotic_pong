#
# testing the comm and controller settings from python to the EPOS 70/10
#
# Note: this only works if the EPOS configuration software is not connected
#

import serial
import time
from ctypes import *
# import winsound
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



# Configure desired motion profile
acceleration = 30000 # rpm/s, up to 1e7 would be possible
deceleration = 30000 # rpm/s

# Query motor position
def GetPositionIs():
    pPositionIs=c_long()
    pErrorCode=c_uint()
    ret=epos.VCS_GetPositionIs(keyHandle, nodeID, byref(pPositionIs), byref(pErrorCode))
        
    #print('got position\n')
    return pPositionIs.value # motor steps

# Move to position at speed
# In general the positioning is not fully accurate so when it reaches a close neighborhood
# of the target point it stops. Stoping for more than 10s terminates the function
def MoveToPositionSpeed(target_position,target_speed):
    previous_position = GetPositionIs()
    flag = 0
    delta_t = 0
    initial_time = time.time()
    while True:
        if target_speed != 0:
            epos.VCS_SetPositionProfile(keyHandle, nodeID, target_speed, acceleration, deceleration, byref(pErrorCode)) # set profile parameters
            epos.VCS_MoveToPosition(keyHandle, nodeID, target_position, True, True, byref(pErrorCode)) # move to position
                
        elif target_speed == 0:
            epos.VCS_HaltPositionMovement(keyHandle, nodeID, byref(pErrorCode)) # halt motor
            print('motion halted\n')

        true_position = GetPositionIs()
        delta_pos = true_position - previous_position
        previous_position = true_position
        
        #print('position {0}   {1}   {2}\n'.format(true_position, delta_t, delta_pos))
        if abs(true_position - target_position)<200:
            break
        elif abs(delta_pos)<10:
            if flag==0:
                #initial_time = time.time()
                flag = 1
            else:
                delta_t = time.time() - initial_time
                if delta_t>10.0:
                    print('bailing out {0}\n'.format(delta_t))
                    break
        elif abs(delta_pos)>10:
            initial_time = time.time()
        

# move the motor 

def MoveBothToPositionSpeed(target_position,target_speed):
    previous_position = GetPositionIs()
    flag = 0
    delta_t = 0
    initial_time = time.time()
    while True:
        if target_speed != 0:
            epos.VCS_SetPositionProfile(keyHandle, nodeID, target_speed, acceleration, deceleration, byref(pErrorCode)) # set profile parameters
            epos.VCS_MoveToPosition(keyHandle, nodeID, target_position, True, True, byref(pErrorCode)) # move to position
                
        elif target_speed == 0:
            epos.VCS_HaltPositionMovement(keyHandle, nodeID, byref(pErrorCode)) # halt motor
            print('motion halted\n')

        true_position = GetPositionIs()
        delta_pos = true_position - previous_position
        previous_position = true_position
        
        #print('position {0}   {1}   {2}\n'.format(true_position, delta_t, delta_pos))
        if abs(true_position - target_position)<200:
            break
        elif abs(delta_pos)<10:
            if flag==0:
                #initial_time = time.time()
                flag = 1
            else:
                delta_t = time.time() - initial_time
                if delta_t>10.0:
                    print('bailing out {0}\n'.format(delta_t))
                    break
        elif abs(delta_pos)>10:
            initial_time = time.time()



def MoveWithVelocity(target_speed,duration):
    #previous_position = GetPositionIs()
    flag = 0
    delta_t = 0
    initial_time = time.time()
    while True:
        if target_speed != 0:
            epos.VCS_SetVelocityProfile(keyHandle, nodeID, acceleration, deceleration, byref(pErrorCode)) # set profile parameters
            epos.VCS_MoveWithVelocity(keyHandle, nodeID, target_velocity, byref(pErrorCode))
                
        elif target_speed == 0:
            epos.VCS_HaltPositionMovement(keyHandle, nodeID, byref(pErrorCode)) # halt motor
            print('motion halted\n')
            break

        delta_t = time.time() - initial_time
        if delta_t > duration:
            target_speed = 0


if __name__ == "__main__":
    EC_PID_POSITION_CONTROLLER = 20
    EC_PIDPC_P_GAIN = 1
    EC_PIDPC_I_GAIN = 2
    EC_PIDPC_D_GAIN = 3

    EC_PI_VELOCITY_CONTROLLER = 10
    EG_PIVC_P_GAIN = 1
    EG_PIVC_I_GAIN = 2

    Kp = 7000
    Ki = 500
    
    #
    # Initiating connection and setting motion profile for EPOS
    #
    keyHandle = epos.VCS_OpenDevice(b'EPOS', b'MAXON_RS232', b'RS232', b'COM22', byref(pErrorCode)) # specify EPOS version and interface
    if keyHandle == 0:
        print('device not open')
        sys.exit()
    
    if epos.VCS_SetProtocolStackSettings(keyHandle, baudrate, timeout, byref(pErrorCode)) == 0:
        print('protocol not set')
        sys.exit()
        
    if epos.VCS_ClearFault(keyHandle, nodeID, byref(pErrorCode)) == 0:
        print('faults not cleared')
       # sys.exit()
        
    #if epos.VCS_ActivateProfilePositionMode(keyHandle, nodeID, byref(pErrorCode)) == 0:
    #    print('profile position not activated')
    if epos.VCS_ActivateVelocityMode(keyHandle, nodeID, byref(pErrorCode)) == 0:
        print('velocity not activated')
        #sys.exit()
    
    if epos.VCS_SetEnableState(keyHandle, nodeID, byref(pErrorCode)) == 0:
        print('state not enabled')
        #sys.exit()

        
    # from pp 59 of the reference manual
    #epos.VCS_SetControllerGain(keyHandle, nodeID, EC_PID_POSITION_CONTROLLER, EC_PIDPC_P_GAIN, 20, byref(pErrorCode))
    #epos.VCS_SetControllerGain(keyHandle, nodeID, EC_PID_POSITION_CONTROLLER, EC_PIDPC_I_GAIN, 0, byref(pErrorCode))
    #epos.VCS_SetControllerGain(keyHandle, nodeID, EC_PID_POSITION_CONTROLLER, EC_PIDPC_D_GAIN, 0, byref(pErrorCode))
    if epos.VCS_SetControllerGain(keyHandle, nodeID, EC_PI_VELOCITY_CONTROLLER, EG_PIVC_P_GAIN,Kp, byref(pErrorCode)) == 0:
        print('P gain not set')
        #sys.exit()
    if epos.VCS_SetControllerGain(keyHandle, nodeID, EC_PI_VELOCITY_CONTROLLER, EG_PIVC_I_GAIN, Ki, byref(pErrorCode)) == 0:
        print('I gain not set')
        #sys.exit()

    print('EPOS initialized\n')


    ##############################################################


    time.sleep(1)

    # Making it move at rpm

    rpm = -1600  # negative = right-2-left, positive=left-2-right,  viewed from the back of the field

    # winsound.Beep(1000,1000)
    
    if epos.VCS_SetVelocityMust(keyHandle, nodeID, rpm, byref(pErrorCode)) == 0:
        print('velocity not set')
        sys.exit()

    print('moving for 10 seconds')
    time.sleep(10)

    # winsound.Beep(2000,1000)
    
    if epos.VCS_SetVelocityMust(keyHandle, nodeID, -rpm, byref(pErrorCode)) == 0:
        print('velocity not set')
        sys.exit()

    print('reversing the movement for 10 seconds')
    time.sleep(10)

    # winsound.Beep(1000,1000)
        
    # Housekeeping
    if epos.VCS_HaltVelocityMovement(keyHandle, nodeID, byref(pErrorCode)) == 0:
        print('couldnt halt velocity 1')    
    
    if epos.VCS_SetDisableState(keyHandle, nodeID, byref(pErrorCode)) == 0: # disable device 1
        print('device not disabled')
    print('device disabled\n')
          
    epos.VCS_CloseDevice(keyHandle, byref(pErrorCode)) # close device 1
    print('device closed\n')
