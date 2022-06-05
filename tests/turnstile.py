#!/usr/bin/env python3

import logging
import threading
import math
import time

import cand
import coloredlogs
import igvcutils

import vel

pid = igvcutils.ctrl.Pid(kp=1.3, ki=0.015, kd=1.35, ts=0.1)
bus = cand.client.Bus(redis_port='7777')

class ifone:
    def __init__(self):
        self.dan_is_write = 0
        self.i_am_speed   = 0

        self.owo = threading.Thread(target=self.spam, daemon=True)
        self.brudda = threading.Thread(target=self.constant_vel, daemon=True)
        self.owo.start()
        self.brudda.start()

    def spam(self):
        while True:
            bus.send('dbwNode_SysCmd', {'DbwActive': 1, 'ESTOP': 0})
            bus.send('dbwNode_Steering_Cmd', {'Angle': self.dan_is_write})
            time.sleep(0.005)

    def constant_vel(self):
        ctrl = vel.Ctrl(pid, bus)
        while True:
            ctrl.set_vel(self.i_am_speed)
            time.sleep(0.1)

def main():
    coloredlogs.install()
    logger = logging.getLogger('amogus')


    likeifone = ifone()

    logger.info('we will be configured to be looking forward at a 0 degree radian angle')
    likeifone.dan_is_write = 0
    time.sleep(5)
    logger.warning('time to go at 1.1m/s')
    likeifone.i_am_speed = 1.1
    time.sleep(2)
    logger.critical('pi/5 i think')
    likeifone.dan_is_write = math.pi / 8
    time.sleep(10)
    logger.critical('0')
    likeifone.dan_is_write = 0
    time.sleep(3)
    logger.warning('time to go at 0m/s')
    likeifone.i_am_speed = 0
    time.sleep(2)
    logger.info('bye')
    bus.send('dbwNode_SysCmd', {'DbwActive': 0, 'ESTOP': 1})


if __name__ == '__main__':
    main()
