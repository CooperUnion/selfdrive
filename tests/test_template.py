#!/usr/bin/env python3

import argparse
import time

import cand


class Test:
    def __init__(self, *, bus: cand.client.Bus):
        self._bus = bus

    def run(self):
        self._bus.send('DBW_Enable', {'enable': 1})

    def end(self):
        self._bus.send('DBW_Enable', {'enable': 0})


def main():
    parser = argparse.ArgumentParser(description='test template')

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
        test.run()
    except:
        pass

    test.end()


if __name__ == '__main__':
    main()
