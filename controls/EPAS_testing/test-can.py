#!/usr/bin/env python3
import ctypes
import can
import odrive
from odrive.enums import *
import time
import numpy as np
from input_thread import Input

# Connect to can bus interface for absolute encoder
bus0 = can.interface.Bus('can0', bustype='socketcan')
time.sleep(1)

print('Connecting to odrive...')
# odrv0 = odrive.find_any(timeout=0.5, serial_number=35550342033494)
odrv0 = odrive.find_any() #TODO: Add serial number here so that only our odrive can be connected
print('Connected to odrive.')

# Get axis from the odrive that is connected.
time.sleep(1)
axis = odrv0.axis0
time.sleep(1)

odrv0.clear_errors() # clear odrive errors before calibration
odrive.utils.dump_errors(odrv0)

axis.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE # Calibrate odrive
time.sleep(1)

# Wait for calibration to finish
while axis.current_state != AXIS_STATE_IDLE: # wait for odrive to finish calibration
   time.sleep(0.1)

time.sleep(2)
print('Ready to go closed loop!')

odrive.utils.dump_errors(odrv0)

axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
axis.controller.config.input_mode = INPUT_MODE_PASSTHROUGH
axis.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL

print(axis.encoder.pos_estimate)

# IMPORTANT CONTROLS FUNCTIONS: ---------------------------------------------------------------------------------------

# Run control loop calculation, return controller output
# Reference is the desired angular position of the steering encoder (currently in raw_counts from absolute encoder)
# Actual is the current position of the steering encoder (currently also in raw_counts from absolute encoder)
def get_controls(reference, actual):
    kp = 0.1 #Proportional gain of the position loop, 
    return (reference-actual) * kp # return the controller output (velocity command to odrive in turns/sec)

#convert the raw_count data from the absolute encoder to a useful unit (either in "turns", "degrees", or "radians"
def counts_to_turns(raw_count):
    ppr = 5760 # pulse per rev of the absolute encoder
    pass

def set_axis_idle():
    axis.requested_state = AXIS_STATE_IDLE



if __name__ == "__main__":
#----------------
    try:
    #Initialize input thread
        key_in = Input()
    #Initialize init and previous position to None
        prev_pos = None
        init_pos = None
        desired_pos = None
        delta_desired_pos = None
        frequency = 0.25 
        t0 = None

    # Control loop
        while 1:
        # Get the absolute encoder data
            rec = bus0.recv() 
            # print(rec.data.hex())
            # print("{} || {}".format(rec.data[2:4], rec.data.hex()))

        # Take bytes 2 and 3, convert to signed integer, they overflow at a werid non-power of two, but it is repeatable.
            pos_masked = int.from_bytes(rec.data[1:3], "big", signed = True) # Convert from byte to ints
            # print("hex: {}".format(rec.data[1:3].hex()))

        #get initial position value to use as our reference, just so we don't spin the motor away from where it currently is.
            if(prev_pos == None): 
                init_pos = pos_masked
                desired_pos = init_pos
                t0 = time.perf_counter()
        
        # calculate sine wave delta position input
            t = time.perf_counter() - t0
            delta_desired_pos = 5000*np.sin(2*np.pi*frequency*t)

        # get input from user, change desired_pos based on user input
            if key_in.get_input() != None:
                if key_in.get_input() == 'w': desired_pos += key_in.step_amount
                elif key_in.get_input() == 's': desired_pos -= key_in.step_amount
                elif key_in.get_input() == 'p': key_in.step_amount += 10
                elif key_in.get_input() == 'o': key_in.step_amount -= 10
                key_in.reset_input()
                print("step amount: " + str(key_in.step_amount))
                
        # get vel command from p controller
            vel_cmd = get_controls(desired_pos, pos_masked)

        #send vel_cmd to odrive but CLIP THE OUTPUT!
            axis.controller.input_vel = np.clip(vel_cmd, -5, 5)
            # axis.controller.input_vel = 0.0

        # Only print data if the encoder has moved
            if(prev_pos != pos_masked): print("raw_counts: " + str(pos_masked) + '| vel_cmd(turns/sec): ' + str(vel_cmd) + ' | ddt: ' + str(delta_desired_pos))

        # Update previous position to the current position
            prev_pos = pos_masked #update previous position value
        
        # Check if odrive axis has any errors!
            if(axis.error != 0): 
                print("Odrive has an error! : Odrive error #: " + str(odrive.error))
                raise Exception

        # Catch the overflow here
        

    except KeyboardInterrupt or Exception as e:
        print("Exception: " + str(e))
        set_axis_idle()
