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

def set_servo_pwm(servo_n, microseconds):
    """ Sets AUX 'servo_n' output PWM pulse-width.

    Uses https://mavlink.io/en/messages/common.html#MAV_CMD_DO_SET_SERVO

    'servo_n' is the AUX port to set (assumes port is configured as a servo).
        Valid values are 1-3 in a normal BlueROV2 setup, but can go up to 8
        depending on Pixhawk type and firmware.
    'microseconds' is the PWM pulse-width to set the output to. Commonly
        between 1100 and 1900 microseconds.

    """
    # master.set_servo(servo_n+8, microseconds) or:
    MAVLINK_CONNECTION.mav.command_long_send(
        MAVLINK_CONNECTION.target_system, MAVLINK_CONNECTION.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
        0,            # first transmission of this command
        servo_n,  # servo instance
        microseconds, # PWM pulse-width
        0,0,0,0,0     # unused parameters
    )


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

############################################################################################3
MAVLINK_CONNECTION.mav.command_long_send(MAVLINK_CONNECTION.target_system, MAVLINK_CONNECTION.target_component, mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0, 1, 0, 0, 0, 0, 0, 0)

MAVLINK_CONNECTION.mav.command_long_send(MAVLINK_CONNECTION.target_system, MAVLINK_CONNECTION.target_component, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 21196, 0, 0, 0, 0, 0)
##################################################################################################

time.sleep(0.1)

# Setup camera gimbal_2D in central position (min: 1100, max: 1900)
set_servo_pwm(5, 1500) # ROLL
set_servo_pwm(6, 1500) # TILT

time.sleep(0.1)

while True:
    # # https://ardupilot.org/dev/docs/copter-commands-in-guided-mode.htmlsssss
    print("thtottle: "+str(throttle)+ "; roll: "+str(roll)+"; pitch: "+str(pitch)+ "; yaw: "+str(yaw))
    MAVLINK_CONNECTION.mav.manual_control_send(MAVLINK_CONNECTION.target_system, roll, pitch, throttle, yaw, 0)
    time.sleep(0.1)

#while True:
#    for us in range(1100, 1900, 50):
#        set_servo_pwm(3, us)
#        set_rc_channel_pwm(7, us)
#        time.sleep(0.1)
#while True:
#    for angle in range(-50, 50):
#        look_at(angle*100)
#        time.sleep(0.1)
#    for angle in range(-50, 50):
#        look_at(-angle*100)
#        time.sleep(0.1)


