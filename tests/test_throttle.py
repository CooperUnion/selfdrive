#!/usr/bin/env python3

import argparse
import time

import cand


DESIRED_VELOCITY_MPS = 4


class Test:
    def __init__(self, *, bus: cand.client.Bus):
        self._bus = bus

    def run(self, percent, duration):
        print('starting throttle')
        while duration > 0:
            self._bus.send(
                'DBW_RawVelocityCommand',
                {
                    'DBW_throttlePercent': min(max(0, percent), 100),
                    'DBW_brakePercent': 0,
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
