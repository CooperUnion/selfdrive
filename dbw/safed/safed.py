#!/usr/bin/env python3

import argparse
import logging
import time

import cand
import coloredlogs

import tests
import unit


DBW_ACTIVE_TIMEOUT_NS = 20_000_000
TEST_DELAY_S = 0.01


class Safed:
    def __init__(self, bus: cand.client.Bus, tests: list[unit.Unit]):
        self._bus = bus
        self._tests = tests

        self._logger = logging.getLogger('safed')

        for test in tests:
            test.bus = self._bus

    def run(self) -> bool:
        rec = self._bus.get('dbwActive')
        if rec is None:
            return True

        msg_unix_ns, data = rec

        if (
            not data['Active']
            or time.time_ns() - msg_unix_ns >= DBW_ACTIVE_TIMEOUT_NS
        ):
            return True

        out = True

        for test in self._tests:
            test_ret = test.test()
            if not test_ret:
                out = False
                self._logger.critical(f'\'{type(test).__name__}\' failed')

        return out


def main():
    argparser = argparse.ArgumentParser(description='sanity checks')

    argparser.add_argument(
        '--redis-host',
        default='localhost',
        help='hostname of redis server',
        metavar='localhost',
    )
    argparser.add_argument(
        '--redis-port',
        default='6379',
        help='port of redis server',
        metavar='6379',
    )

    args = argparser.parse_args()

    coloredlogs.install()
    logger = logging.getLogger('safed')

    bus = cand.client.Bus(
        redis_host=args.redis_host,
        redis_port=args.redis_port,
    )

    safed = Safed(bus, tests.units)

    logger.info('initialization complete')

    while True:
        if not safed.run():
            logger.error('unit test(s) failed')

        time.sleep(TEST_DELAY_S)


if __name__ == '__main__':
    main()
