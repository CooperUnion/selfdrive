#!/usr/bin/env python3

import argparse
import time

import cand

# from ctypes import (
#     c_int16,
#     c_uint16,
# )


DESIRED_VELOCITY_MPS = 4


class Test:
    def __init__(self, *, bus: cand.client.Bus):
        self._bus = bus

    def _desired_velocity_check(self):
        _, data = self._bus.get('CTRL_ControllerData')

        return data['CTRL_averageVelocity'] < DESIRED_VELOCITY_MPS

    def run(self, percent, duration):
        print('starting test')

        while self._desired_velocity_check():
            self._bus.send(
                'DBW_RawVelocityCommand',
                {
                    'DBW_throttlePercent': 70,
                    'DBW_brakePercent': 0,
                },
            )
            time.sleep(0.005)

        print('starting brake')
        while duration > 0:
            self._bus.send(
                'DBW_RawVelocityCommand',
                {
                    'DBW_throttlePercent': 0,
                    'DBW_brakePercent': min(max(0, percent), 100),
                },
            )

            time.sleep(0.005)
            duration -= 0.005

    def end(self):
        pass


def main():
    parser = argparse.ArgumentParser(description='test throttle')

    parser.add_argument(
        '-p',
        '--percent',
        help='value within 0.0 and 100.0',
        metavar='n',
        type=float,
        required=True,
    )
    parser.add_argument(
        '-t',
        '--time',
        help='duration in seconds',
        metavar='n.n',
        type=float,
        required=True,
    )
    parser.add_argument(
        '--redis-host',
        default='localhost',
        help='redis hostname',
        metavar='localhost',
    )
    parser.add_argument(
        '--redis-port',
        default='6379',
        help='redis hostname',
        metavar='6379',
    )

    args = parser.parse_args()

    bus = cand.client.Bus(
        redis_host=args.redis_host,
        redis_port=args.redis_port,
    )

    test = Test(
        bus=bus,
    )

    try:
        test.run(args.percent, args.time)
    except Exception:
        pass

    test.end()


if __name__ == '__main__':
    main()
