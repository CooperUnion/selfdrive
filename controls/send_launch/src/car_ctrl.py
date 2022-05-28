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

from igvcutils.can import endianswap



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
        return (-49.13*brake)
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

    def __init__(self, kp=0.0, ki=0.0, kd=0.0, Ts=0.1, flag=False):
        self.controller = pid.Controller(kp=kp, ki=0, kd=0, ts=Ts, lower_lim=-10, upper_lim= 10)
        self.bus = Bus(redis_host='redis')
        self.enc_count=0
        self.ang_filtered = 0                                                                       #filtering angle,, i just copied and pasted and changed some stuff from vel_ctrl ;p
        self.N            = 4
        self.ang_history = np.zeros(self.N)
        self.candListener = Thread(target=self.steering_enc_filter, daemon=True)
        self.candListener.start()
        self.odrive_calibrated=flag

        # Connect to can bus interface for absolute encoder

        time.sleep(1)

        print('Connecting to odrive...')
        # odrv0 = odrive.find_any(timeout=0.5, serial_number=35550342033494)
        odrv0 = odrive.find_any() #TODO: Add serial number here so that only our odrive can be connected
        print('Connected to odrive.')
        # Get axis from the odrive that is connected.
        time.sleep(1)
        self.axis = odrv0.axis0
        time.sleep(1)
    
        print(f"errors: {self.axis.error}")

        if not self.axis.motor.is_calibrated or \
            self.axis.error or \
            self.axis.motor.error or \
            self.axis.sensorless_estimator.error or \
            self.axis.encoder.error or \
            self.axis.controller.error:
            odrive.utils.dump_errors(odrv0)
            odrv0.clear_errors() # clear odrive errors before calibration
            odrive.utils.dump_errors(odrv0)

            self.axis.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE # Calibrate odrive
            time.sleep(1)

            # Wait for calibration to finish
            while self.axis.current_state != AXIS_STATE_IDLE: # wait for odrive to finish calibration
                time.sleep(0.1)

        self.axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

        self.axis.controller.config.input_mode = INPUT_MODE_PASSTHROUGH
        self.axis.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL


        if odrive.utils.dump_errors(odrv0) !=0: #check before cont'd
            self.flag=True

    def steering_enc_filter(self): 
        prev_time = 0
        index = 0

        while True:
            msg_time, data = self.bus.get('steering_Absolute_Encoder')
            if data is None:
                print(f"WARNING: NO ENCODER DATA")

            time_delta_ns = time.time_ns() - msg_time
            time_delta_ms = time_delta_ns / 1_000_000

            if time_delta_ms > 1000:
                print(f"ERROR: CAN ENCODER DATA TIMEOUT")
                self.axis.controller.input_vel = 0
                self.axis.requested_state = AXIS_STATE_IDLE
                exit(-5)
                # trip ESTOP

            #print(f"CAN Encoder data: {data}")
            # self.enc_count= int.from_bytes(msg.data[1:3], 'big', signed=True)
            self.enc_count = endianswap(data['Pos'], 'little', dst_signed=True)

    def enc_to_angle(self, count):
        self.angle = 0.0029*count + 0.0446
        return (self.angle)
    def ctrl_vel_twist(self, msg: Twist):
        print(f"angular.z: {msg.angular.z}")
        theta_des = ((msg.angular.z)*(180/np.pi))+(np.pi/2) # Get desired angle from Twist message
        if theta_des >27:
            theta_des=27
        elif theta_des<-27:
            theta_des = -27
        theta_actual = self.enc_to_angle(self.enc_count) #define angle_filtered in enc_filter
        self.PID_out = self.controller.step(theta_des , theta_actual)
        self.axis.controller.input_vel = self.PID_out
        print(f"td:{theta_des} | enc_count: {self.enc_count} | ta:{theta_actual} | PID: {self.PID_out}")

if __name__ == "__main__":
    try:
        ang=angle_ctrl(kp=1.75)
        t0 = time.time_ns()

        while True:
            t1 = (time.time_ns() - t0)/1_000_000_000
            msg=Twist()
            msg.angular.z= -(np.pi/6)
            ang.ctrl_vel_twist(msg)
            time.sleep(ang.controller.ts)
    except:
        ang.axis.controller.input_vel = 0
        ang.axis.requested_state = AXIS_STATE_IDLE

    # parser = argparse.ArgumentParser(description="test throttle")

    # parser.add_argument(
    #     "--kp",
    #     metavar="n",
    #     type=float,
    #     required=True,
    # )
    # parser.add_argument(
    #     "--ki",
    #     metavar="n",
    #     type=float,
    #     required=True,
    # )
    # parser.add_argument(
    #     "--kd",
    #     metavar="n",
    #     type=float,
    #     required=True,
    # )
    # parser.add_argument(
    #     "--vd",
    #     metavar="n",
    #     type=float,
    #     required=True,
    # )

    # args = parser.parse_args()
    # c = vel_ctrl(args.kp, args.ki, args.kd)
    # count = 0
    # vd = args.vd
    # while(1):
    #     c.ctrl_vel_fixed(vd)
    #     time.sleep(0.1)
    #     count += 1
    #     if count > 60:
    #         vd = 0

    #     if count > 120:
    #         vd = 0

    #     if count > 180:
    #         vd = 0
