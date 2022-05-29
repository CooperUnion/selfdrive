#!/usr/bin/env python3

import argparse
import sys

import cand
import igvcutils
import rospy

import pid
import twist
import vel


def main():
    argparser = argparse.ArgumentParser(description='send_launch')

    argparser.add_argument(
        '--kp',
        default=0.0,
        metavar='n',
        type=float,
    )
    argparser.add_argument(
        '--ki',
        default=0.0,
        metavar='n',
        type=float,
    )
    argparser.add_argument(
        '--kd',
        default=0.0,
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

    ctrl = vel.Ctrl(
        igvcutils.ctrl.Pid(
            kp=args.kp,
            ki=args.ki,
            kd=args.kd,
            ts=(1 / args.rate),
        ),
        cand.client.Bus(redis_host='redis'),
    )
    decoder = twist.Decode('/cmd_vel')

    rospy.init_node('send_launch')

    r = rospy.Rate(args.rate)
    while not rospy.is_shutdown():
        ctrl.set_vel(decoder.vel)
        r.sleep()


if __name__ == '__main__':
    main()
