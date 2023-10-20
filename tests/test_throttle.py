#!/usr/bin/env python3

import argparse
import time

import cand


class Test:
    def __init__(self, *, bus: cand.client.Bus):
        self._bus = bus

    def run(self, percent, duration):
        self._bus.send('DBW_Enable', {'DBW_enable': 1})
        self._bus.send(
            'DBW_VelCmd',
            {
                'DBW_throttlePercent': min(abs(percent), 100),
                'DBW_brakePercent': 0,
            },
        )

        time_start = time.time()

        i = 0
        while True:
            if (time.time() - time_start) > abs(duration):
                break

            print(f"{i} {self._bus.get_data('ENCF_EncoderData')}")

            i += 1
            time.sleep(0.01)

    def end(self):
        self._bus.send(
            'DBW_VelCmd',
            {'DBW_throttlePercent': 0, 'DBW_brakePercent': 0},
        )
        self._bus.send('DBW_Enable', {'DBW_enable': 0})


def main():
    parser = argparse.ArgumentParser(description='test throttle')

    parser.add_argument(
        '-p',
        '--percent',
        help='value within 0 and 100',
        metavar='n',
        type=int,
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
