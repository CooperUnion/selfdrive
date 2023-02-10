#!/usr/bin/env python3

import argparse
import sys
import threading
import time

import cand
import igvcutils

import vel


class VelocityControl(threading.Thread):
    SEND_RATE_S = 0.01

    def __init__(self, *, bus: cand.client.Bus = None):
        self._bus      = bus

        self.throttle_percent = 0
        self.brake_percent    = 0
        self.steering_angle   = 0.0

        super().__init__(daemon=True)

    def run(self):
        while True:
            self._bus.send(
                'DBW_RawVelocityCommand',
                {
                    'DBW_brakePercent':    self.brake_percent,
                    'DBW_throttlePercent': self.throttle_percent,
                },
            )
            time.sleep(self.SEND_RATE_S)


def main():
    argparser = argparse.ArgumentParser(description='velocity controller')

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

    args = argparser.parse_args()
    args.rate = abs(args.rate)

    ts = 1 / args.rate

    bus = cand.client.Bus()

    pid = vel.Ctrl(
        igvcutils.ctrl.Pid(
            kp=args.kp,
            ki=args.ki,
            kd=args.kd,
            ts=ts,
        ),
        bus=bus,
    )

    ctrl = VelocityControl(
        bus=bus,
    )
    ctrl.start()

    while True:
        lin_vel = 0

        rec = bus.get('DBW_VelocityCommand')
        if rec:
            unix_time_ns, data = rec

            if time.time_ns() - unix_time_ns <= 200_000_000:
                lin_vel = data['linearVelocity']

        throttle_percent, brake_percent = pid.set_vel(lin_vel)

        ctrl.brake_percent    = brake_percent
        ctrl.throttle_percent = throttle_percent

        time.sleep(ts)


if __name__ == '__main__':
    main()
