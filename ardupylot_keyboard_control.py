import sys
import time
from pymavlink import mavutil
from pynput import keyboard
import math


MAVLINK_CONNECTION = mavutil.mavlink_connection('udpin:localhost:14550', baud=1500000) 

MAVLINK_CONNECTION.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" % (MAVLINK_CONNECTION.target_system, MAVLINK_CONNECTION.target_component))
  
##################################################################################################################
# Manual Control Variables
roll, pitch, throttle, yaw = 0, 0, 500, 0
##################################################################################################################  


def on_press(key):
    global roll, pitch, throttle, yaw

    roll, pitch, throttle, yaw = 0, 0, 500, 0
    value = 600
    
    try:
        
        if key.char == 'r': #forward back
            pitch = -value
        elif key.char == 'f':
            pitch = value
        if key.char == 't': #left rogh
            roll = value
        elif key.char == 'g':
            roll = -value
        if key.char == 'w':
            throttle = value #throtle
        elif key.char == 's':
            throttle = -value
        if key.char == 'a':
            yaw = -value # rotate
        elif key.char == 'd':
            yaw = value
        
        if key.char == 'q':
            print("target is left")
            roll, pitch, throttle, yaw = 0, 0, 500, 0
        elif key.char == 'o':
            print("MODE: pos hold")
            MAVLINK_CONNECTION.mav.command_long_send(MAVLINK_CONNECTION.target_system, MAVLINK_CONNECTION.target_component, mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0, 1, 16, 0, 0, 0, 0, 0) 
        elif key.char == 'p':
            print("MODE: guided no gps")
            MAVLINK_CONNECTION.mav.command_long_send(MAVLINK_CONNECTION.target_system, MAVLINK_CONNECTION.target_component, mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0, 1, 20, 0, 0, 0, 0, 0)       
        elif key.char == 'l':
            print("exit")
            sys.exit("Exiting the code with sys.exit()!")



    except AttributeError:
        print('special key {0} pressed'.format(
            key))
            
        
listener = keyboard.Listener(on_press=on_press)
listener.start()

# TODO: Maybe not need! ArduPilot Should calculate home by it selfe, even with optic flow 
# Manual Set HOME 
#MAVLINK_CONNECTION.mav.command_long_send(MAVLINK_CONNECTION.target_system, MAVLINK_CONNECTION.target_component, mavutil.mavlink.MAV_CMD_DO_SET_HOME , 0, 0, 4, 0, 0, -35.363, 149.165, 100)

# TODO: Need set GUIDED_NOGPS MODE
# Set GUIDED mode
#MAVLINK_CONNECTION.mav.command_long_send(MAVLINK_CONNECTION.target_system, MAVLINK_CONNECTION.target_component, mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0, 1, 4, 0, 0, 0, 0, 0)

# TODO: Next Time try this:
#MAVLINK_CONNECTION.mav.command_long_send(MAVLINK_CONNECTION.target_system, MAVLINK_CONNECTION.target_component, mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0, 1, 20, 0, 0, 0, 0, 0)
MAVLINK_CONNECTION.mav.command_long_send(MAVLINK_CONNECTION.target_system, MAVLINK_CONNECTION.target_component, mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0, 1, 0, 0, 0, 0, 0, 0)

MAVLINK_CONNECTION.mav.command_long_send(MAVLINK_CONNECTION.target_system, MAVLINK_CONNECTION.target_component, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 21196, 0, 0, 0, 0, 0)

while True:
    # # https://ardupilot.org/dev/docs/copter-commands-in-guided-mode.htmlsssss
    print("thtottle: "+str(throttle)+ "; roll: "+str(roll)+"; pitch: "+str(pitch)+ "; yaw: "+str(yaw))
    MAVLINK_CONNECTION.mav.manual_control_send(MAVLINK_CONNECTION.target_system, roll, pitch, throttle, yaw, 0)
    time.sleep(0.1)
    


