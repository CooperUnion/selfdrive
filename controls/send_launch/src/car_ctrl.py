#!/usr/bin/env python3
# ./vel_ctrl.py --kp 1.3 --ki 0.015 --kd 1.35 --vd 2
import argparse
import time
import numpy as np
from cand.client import Bus
import pid

from threading import Thread

from geometry_msgs.msg import Twist

import ctypes
import odrive
from odrive.enums import *

from input_thread import Input


class vel_ctrl:

    def __init__(self, kp=0.0, ki=0.0, kd=0.0, Ts=0.1):
        self.controller = pid.Controller(kp=kp, ki=ki, kd=kd, ts=Ts)
        self.bus = Bus(redis_host='redis')
        self.vel_filtered = 0
        self.N            = 4
        self.vel_history = np.zeros(self.N)
        self.candListener = Thread(target=self.vel_filter, daemon=True)
        self.candListener.start()

    def vel_filter(self):

        prev_time = 0
        index = 0

        while True:
            data = self.bus.get('dbwNode_Encoder_Data')

            if prev_time == data[0]:
                continue

            prev_time = data[0]

            self.vel_history[index] = vel_ctrl.enc_to_velocity((data[1]['Encoder0'] + data[1]['Encoder1']) / 2, 0.01)
            vel_in = self.vel_history[index]
            index = (index + 1) % self.N

            self.vel_filtered = np.mean(self.vel_history)

    def test_connection(self, twist_message, ctrl_out):
        v_des = twist_message.linear.x
        pedal_percentage = v_des/100
        self.bus.send("dbwNode_SysCmd", {"DbwActive": 1, "ESTOP": 0})
        self.bus.send('dbwNode_Accel_Cmd', {'ThrottleCmd': max(pedal_percentage, 0) / 100, 'ModeCtrl': 1})


#        self.bus.send('dbwNode_Accel_Cmd', {'ThrottleCmd': max(pedal_percentage, 0) / 100, 'ModeCtrl': 1})
#        def ctrl_from_twist(self, twist_message):
#        v_des = twist_message.linear.x
#        enc = self.bus.get('dbwNode_Encoder_Data')
#        delta = (time.time_ns() - enc[0])/1000000000
#        data = enc[1]
#        v_actual = self.enc_to_velocity(data, delta)
#        ctrl_out = self.controller.step(v_des, v_actual)
#        pedal_percentage = self.acc_to_pedal(ctrl_out)
#        print(f'v_actual: {v_actual} pedal_percentage: {pedal_percentage}')
#        self.bus.send("dbwNode_SysCmd", {"DbwActive": 1, "ESTOP": 0})
#        self.bus.send('dbwNode_Accel_Cmd', {'ThrottleCmd': max(pedal_percentage, 0) / 100, 'ModeCtrl': 1})

    def ctrl_vel_twist(self, msg: Twist):
        v_des = msg.linear.x # Get desired velocity from Twist message
        v_actual = self.vel_filtered
        acceleration_desired = self.controller.step(v_des, self.vel_filtered)
        (throttle_percentage, brake_percentage) = self.brake_or_throttle(v_actual, acceleration_desired)
        print(v_actual)
        #print(throttle_percentage)
        self.bus.send("dbwNode_SysCmd", {"DbwActive": 1, "ESTOP": 0})
        self.bus.send('dbwNode_Accel_Cmd', {'ThrottleCmd': min(max(throttle_percentage, 0), 100) / 100, 'ModeCtrl': 1})
        self.bus.send('dbwNode_Brake_Cmd', {'BrakeCmd': min(max(brake_percentage, 0), 100) / 100})

    def ctrl_vel_fixed(self, v_des=2.2352):
        #enc = self.bus.get('dbwNode_Encoder_Data')
        #delta = (time.time_ns() - enc[0])/1_000_000_000
        #data = enc[1]
        v_actual = self.vel_filtered
        acceleration_desired = self.controller.step(v_des, self.vel_filtered)
        (throttle_percentage, brake_percentage) = self.brake_or_throttle(v_actual, acceleration_desired)
        print(v_actual)
        #print(throttle_percentage)
        self.bus.send("dbwNode_SysCmd", {"DbwActive": 1, "ESTOP": 0})
        self.bus.send('dbwNode_Accel_Cmd', {'ThrottleCmd': min(max(throttle_percentage, 0), 100) / 100, 'ModeCtrl': 1})
        self.bus.send('dbwNode_Brake_Cmd', {'BrakeCmd': min(max(brake_percentage, 0), 100) / 100})

    def acc_to_pedal(self, acceleration):
        return 15.4*acceleration

    def signal_processing(self, v_sample):
        pass

    @staticmethod
    def enc_to_velocity(ticks, time):
        enc_ticks = 4000
        wheel_circumference = 1.899156
        meters_per_tick = wheel_circumference/enc_ticks
        return (meters_per_tick * ticks)/time

    def brake_to_pedal(self, brake):
        return (-58.03*brake)-11.33
    # def brake_or_throttle(self, v_des, v_actual, acceleration_desired):
    #         if (not (v_des == v_actual)):

    #             if((v_actual>= 0 and v_des>= 0) or (v_actual<= 0 and v_des<= 0)):
    #                 if(np.abs(v_actual)>np.abs(v_des) and acceleration_desired):
    #                     return (0, self.brake_to_pedal(acceleration_desired))

    #                 else:
    #                     return (self.acc_to_pedal(acceleration_desired), 0)

    #             elif ((v_actual<= 0 and v_des>= 0) or (v_actual>= 0 and v_des <= 0)):
    #                 return(0, self.brake_to_pedal(acceleration_desired))
    #         elif(acceleration_desired>=0):
    #             return (self.acc_to_pedal(acceleration_desired), 0)
    #         elif(acceleration_desired<0):
    #             return(0, self.brake_to_pedal(acceleration_desired))
    def brake_or_throttle(self, v_actual, acceleration_desired):
        if(v_actual<0):
            return(0, 60)
        elif(v_actual>= 0):
            if(acceleration_desired>0):
                return (self.acc_to_pedal(acceleration_desired), 0)
            elif(acceleration_desired<=0):
                return(0, self.brake_to_pedal(acceleration_desired))


class angle_ctrl:

    def __init__(self, kp=0.0, ki=0.0, kd=0.0, Ts=0.1):
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

        self.controller = pid.Controller(kp=kp, ki=ki, kd=kd, ts=Ts)
        self.bus = Bus(redis_host='redis')
        self.ang_filtered = 0                                                                       #filtering angle,, i just copied and pasted and changed some stuff from vel_ctrl ;p
        self.N            = 4
        self.ang_history = np.zeros(self.N)
        self.candListener = Thread(target=self.ang_filter, daemon=True)
        self.candListener.start()

        axis = odrv0.axis0
        odrive.utils.dump_errors(odrv0)


    def abs_enc_filter(self): #ASK JACOB TO DO THIS
        prev_time = 0
        index = 0

        while True:
            data = self.bus.get('wherever the thing fpr the steering enc is ')                      #DONT FORGET TO FIX THIS can message for abs enc

            if prev_time == data[0]:
                continue

            prev_time = data[0]

            self.ang_history[index] = angle_ctrl.enc_to_angle((data[1]['Encoder0'] + data[1]['Encoder1']) / 2, 0.01)        #AND FIX THIS TO REFLECT WHATEWVRE IS GOING ON IN THE WHEELS WHEN TURNING IDEK
            ang_in = self.ang_history[index]
            index = (index + 1) % self.N

            self.vel_filtered = np.mean(self.vel_history)

    def enc_to_angle(self, data):
        self.angle = data*data*(2*(10**(-7)))
        return (some mapping*self.enc)                                                                                                       #this might be wrong LOL (it probably is bc she isnt linear :P)

    def ctrl_vel_twist(self, msg: Twist):
        theta_des = msg.angular.z # Get desired angle from Twist message
        theta_actual = self.vel_filtered
        self.PID_out = self.controller.run(theta_des, self.vel_filtered)
        (throttle_percentage, brake_percentage) = self.brake_or_throttle(theta_actual, acceleration_desired)

        self.bus.send("dbwNode_SysCmd", {"DbwActive": 1, "ESTOP": 0})
        self.bus.send('dbwNode_Accel_Cmd', {'ThrottleCmd': min(max(throttle_percentage, 0), 100) / 100, 'ModeCtrl': 1})


    def ctrl_vel_fixed(self, theta_des=2.2352):
        theta_actual = self.ang_filtered
        vel_desired = self.controller.run(theta_des, self.ang_filtered)

        #print(throttle_percentage)
        self.bus.send("dbwNode_SysCmd", {"DbwActive": 1, "ESTOP": 0})
        axis.controller.input_vel = self.PID_out





if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="test throttle")

    parser.add_argument(
        "--kp",
        metavar="n",
        type=float,
        required=True,
    )
    parser.add_argument(
        "--ki",
        metavar="n",
        type=float,
        required=True,
    )
    parser.add_argument(
        "--kd",
        metavar="n",
        type=float,
        required=True,
    )
    parser.add_argument(
        "--vd",
        metavar="n",
        type=float,
        required=True,
    )

    args = parser.parse_args()
    c = vel_ctrl(args.kp, args.ki, args.kd)
    count = 0
    vd = args.vd
    while(1):
        c.ctrl_vel_fixed(vd)
        time.sleep(0.1)
        count += 1
        if count > 60:
            vd = 0

        if count > 120:
            vd = 0

        if count > 180:
            vd = 0
