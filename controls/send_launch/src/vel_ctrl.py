#!/usr/bin/env python3
import argparse
import igvcutils
import time
import numpy as np
from cand.client import Bus
from PID_beard import PIDController

#from geometry_msgs.msg import Twist

class vel_ctrl:

    def __init__(self, kp=0.0, ki=0.0, kd=0.0, Ts=0.1):
        self.controller = PIDController(kp=kp, ki=ki, kd=kd, Ts=Ts)
        self.bus = Bus()
        self.vel_history = np.zeros(10)
        self.index = 0

#    def ctrl_from_twist(self, twist_message):
#        v_des = twist_message.linear.x
#        enc = self.bus.get('dbwNode_Encoder_Data')
#        delta = (time.time_ns() - enc[0])/1000000000
#        data = enc[1]
#        v_actual = self.enc_to_velocity(data, delta)
#        ctrl_out = self.controller.PID(v_des, v_actual)
#        pedal_percentage = self.acc_to_pedal(ctrl_out)
#        print(f'v_actual: {v_actual} pedal_percentage: {pedal_percentage}')
#        self.bus.send("dbwNode_SysCmd", {"DbwActive": 1, "ESTOP": 0})
#        self.bus.send('dbwNode_Accel_Cmd', {'ThrottleCmd': max(pedal_percentage, 0) / 100, 'ModeCtrl': 1})

    def ctrl_vel_fixed(self, v_des=2.2352):
        enc = self.bus.get('dbwNode_Encoder_Data')
        delta = (time.time_ns() - enc[0])/1_000_000_000
        data = enc[1]
        v_actual = self.enc_to_velocity(data, delta)
        self.vel_history[self.index] = v_actual
        self.index = (self.index + 1) % 10
        ctrl_out = self.controller.PID(v_des, np.mean(self.vel_history))
        pedal_percentage = self.acc_to_pedal(ctrl_out)
        print(f'np.mean(vel_history): {np.mean(self.vel_history)} pedal_percentage: {pedal_percentage}')
        self.bus.send("dbwNode_SysCmd", {"DbwActive": 1, "ESTOP": 0})
        self.bus.send('dbwNode_Accel_Cmd', {'ThrottleCmd': max(pedal_percentage, 0) / 100, 'ModeCtrl': 1})

    def acc_to_pedal(self, acceleration):
        return 15.4*acceleration

    def enc_to_velocity(self, enc, time):
        enc_ticks = 4000
        wheel_circumference = 1.899156
        meters_per_tick = wheel_circumference/enc_ticks
        return (meters_per_tick * enc['Encoder0'])/time

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
    while(1):
        c.ctrl_vel_fixed(args.vd)
        time.sleep(0.1)
