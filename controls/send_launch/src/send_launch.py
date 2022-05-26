#!/usr/bin/env python3

import argparse

import cand

import pid
import vel


def main():
    argparser = argparse.ArgumentParser(description='send_launch')

    argparser.add_argument(
        "--kp",
        default=0.0,
        metavar="n",
        type=float,
    )
    argparser.add_argument(
        "--ki",
        default=0.0,
        metavar="n",
        type=float,
    )
    argparser.add_argument(
        "--kd",
        default=0.0,
        metavar="n",
        type=float,
    )
    argparser.add_argument(
        "--vd",
        default=0.0,
        metavar="n",
        type=float,
    )

    args = argparser.parse_args()

    ctrl = vel.Ctrl(
        pid.Controller(
            kp=args.kp,
            ki=args.ki,
            kd=args.kd,
        ),
        cand.client.Bus(redis_host='redis'),
    )


if __name__ == '__main__':
    main()
