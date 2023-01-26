#!/usr/bin/env python3
import argparse
import logging

import cand
import coloredlogs
import igvcutils

import base
import steer


def main():
    MOD_IDENT = 'STEER'

    argparser = argparse.ArgumentParser(description='py-steering')
    argparser.add_argument(
        '--redis-host',
        default='localhost',
        help='redis hostname',
        metavar='hostname',
    )
    argparser.add_argument(
        '--redis-port',
        default='6379',
        help='redis port',
        metavar='6379',
    )
    argparser.add_argument(
        '--kp',
        default=1.75,
        metavar='n',
        type=float,
    )
    argparser.add_argument(
        '--ki',
        default=0,
        metavar='n',
        type=float,
    )
    argparser.add_argument(
        '--kd',
        default=0,
        metavar='n',
        type=float,
    )

    args = argparser.parse_args()

    logger = logging.getLogger('py-steering')
    coloredlogs.install()

    bus = cand.client.Bus(
        redis_host=args.redis_host,
        redis_port=args.redis_port,
    )

    logger.info('beginning initialization')

    based = base.Base(
        bus=bus,
        mod_ident=MOD_IDENT,
    )

    pid = igvcutils.ctrl.Pid(
        kp=args.kp,
        ki=args.ki,
        kd=args.kd,
        ts=0.01,
    )

    fidget_spinner = steer.Steer(
        bus=bus,
        pid=pid,
        base=based,
    )

    logger.info('initialization complete')

    based.start()
    fidget_spinner.start()

    try:
        based.join()
        fidget_spinner.join()
    # we want for our ODrive to be disabled on script failure so that we
    # don't need to reset the device to regain manual control
    except:
        fidget_spinner.disable_odrive()


if __name__ == '__main__':
    main()
