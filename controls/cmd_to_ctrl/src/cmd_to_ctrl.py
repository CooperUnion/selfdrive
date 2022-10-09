#!/usr/bin/env python3

import argparse
import coloredlogs
import logging
import sys
import threading
import time

import cand
import igvcutils
import rospy

import twist
import vel


class CmdToCtrl(threading.Thread):
    SEND_RATE_S = 0.001

    def __init__(self, ctrl, decoder, *, bus: cand.client.Bus = None):
        self._log     = logging.getLogger('cmd_to_ctrl')
        self._bus     = bus
        self._ctrl    = ctrl
        self._decoder = decoder

        self.throttle_percent = 0
        self.brake_percent    = 0
        self.steering_angle   = 0

        super().__init__(daemon=True)

    def _run_10Hz(self):
        self._log.info('in run_10Hz loop')
        throttle_percent, brake_percent = self._ctrl.set_vel(self._decoder.vel)

        self.throttle_percent = throttle_percent
        self.brake_percent    = brake_percent
        self.steering_angle   = self._decoder.angle

    def run(self):
        counter = 0

        while True:
            if counter % 10 == 0:
                self._run_10Hz()
                counter = 0

            self._bus.send(
                'dbwNode_Vel_Cmd',
                {
                    'ThrottlePercent': self.throttle_percent,
                    'BrakePercent':    self.brake_percent,
                },
            )
            self._bus.send(
                'dbwNode_Steering_Cmd',
                {
                    'Angle': self.steering_angle,
                },
            )
            self._log.info('Sent CAN commands!')
            time.sleep(self.SEND_RATE_S)


def main():
    coloredlogs.install(level='debug')
    log = logging.getLogger('cmd_to_ctrl')

    log.info('cmd_to_ctrl starting')

    argparser = argparse.ArgumentParser(description='cmd_to_ctrl')

    argparser.add_argument(
        '--kp',
        default=1.3,
        metavar='n',
        type=float,
    )
    argparser.add_argument(
        '--ki',
        default=0.015,
        metavar='n',
        type=float,
    )
    argparser.add_argument(
        '--kd',
        default=1.35,
        metavar='n',
        type=float,
    )
    argparser.add_argument(
        '--vd',
        default=0.0,
        metavar='n',
        type=float,
    )
    argparser.add_argument(
        '-r',
        '--rate',
        default=10.0,
        metavar='Hz',
        type=float,
    )

    args = argparser.parse_args(rospy.myargv(argv=sys.argv)[1:])
    args.rate = abs(args.rate)

    bus = cand.client.Bus(redis_host='redis')

    ctrl = vel.Controller(
        igvcutils.ctrl.Pid(
            kp=args.kp,
            ki=args.ki,
            kd=args.kd,
            ts=(1 / args.rate),
        ),
        bus=bus,
    )

    log.info('created cand client and PID instance')

    decoder = twist.Decode('/cmd_vel')

    log.info('started ROS node')

    rospy.init_node('cmd_to_ctrl')

    cmd_to_ctrl = CmdToCtrl(
        bus=bus,
        ctrl=ctrl,
        decoder=decoder,
    )
    cmd_to_ctrl.start()

    log.info('created cmd_to_ctrl and started')

    rospy.spin()


if __name__ == '__main__':
    main()
