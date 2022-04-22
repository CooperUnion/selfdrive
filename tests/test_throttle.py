#!/usr/bin/env python3

import argparse
import time

import igvcutils


def main():
    parser = argparse.ArgumentParser(description='test throttle')

    parser.add_argument(
        '-c',
        '--can',
        help='CAN device',
        metavar='canx',
    )
    parser.add_argument(
        '-d',
        '--dbc',
        help='CAN DBC',
        metavar='file.dbc',
    )
    parser.add_argument(
        '-p',
        '--percent',
        help='value between 0 and 100',
        metavar='n',
        type=int,
    )
    parser.add_argument(
        '-t',
        '--time',
        help='duration in seconds',
        metavar='n.n',
        type=float,
    )

    args = parser.parse_args()

    bus = igvcutils.can.Bus(args.dbc, args.can)

    bus.send('dbwNode_SysCmd', {'DbwActive': 1, 'ESTOP': 0})
    bus.send('dbwNode_Accel_Cmd', {'ThrottleCmd': min(args.percent, 100) / 100, 'ModeCtrl': 1})
    bus.send('dbwNode_Accel_Cntrls_Cmd', {'TargetVel': 0 ,'Kp': 0, 'Ki': 0,'Kd': 0, 'CharMode': 1})
    time.sleep(args.time)
    bus.send('dbwNode_Accel_Cmd', {'ThrottleCmd': 0, 'ModeCtrl': 1})
    bus.send('dbwNode_SysCmd', {'DbwActive': 0, 'ESTOP': 0})


if __name__ == '__main__':
    main()
