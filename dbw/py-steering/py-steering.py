#!/usr/bin/env python3
import argparse

import cand
import igvcutils

import base
import steer


def main():
    MOD_IDENT = 'Steering'

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

    bus = cand.client.Bus(
        redis_host=args.redis_host,
        redis_port=args.redis_port,
    )

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

    based.start()
    fidget_spinner.start()

    based.join()
    fidget_spinner.join()


if __name__ == '__main__':
    main()
