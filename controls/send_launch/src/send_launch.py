#!/usr/bin/env python3

import argparse
import sys
import threading

import cand
import igvcutils
import rospy

import twist
import vel


class SendLaunch(threading.Thread):
    SEND_RATE_S = 0.01

    def __init__(self, *, bus: cand.client.Bus = None):
        self._bus      = bus

        self.throttle_percent = 0
        self.brake_percent    = 0
        self.steering_angle   = 0.0

        super().__init__(daemon=True)

    def run(self):
        while True:
            self._bus.send(
                'dbwNode_Vel_Cmd',
                {
                    'ThrottlePercent': self.throttle_percent,
                    'BrakePercent':    self.brake_percent,
                },
            )
            self._bus.send(
                'dbwNode_Steering_Cmd',
                {
                    'Angle': self.steering_angle,
                },
            )
            time.sleep(self.SEND_RATE_S)


def main():
    argparser = argparse.ArgumentParser(description='send_launch')

    argparser.add_argument(
        '--kp',
        default=1.3,
        metavar='n',
        type=float,
    )
    argparser.add_argument(
        '--ki',
        default=0.015,
        metavar='n',
        type=float,
    )
    argparser.add_argument(
        '--kd',
        default=1.35,
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

    bus = cand.client.Bus(redis_host='redis')

    ctrl = vel.Ctrl(
        igvcutils.ctrl.Pid(
            kp=args.kp,
            ki=args.ki,
            kd=args.kd,
            ts=(1 / args.rate),
        ),
        bus=bus,
    )
    decoder = twist.Decode('/cmd_vel')

    send_launch = SendLaunch(
        bus=bus,
        vel_ctrl=vel_ctrl,
        decoder=decoder,
    )
    send_launch.start()

    rospy.init_node('send_launch')

    r = rospy.Rate(args.rate)
    while not rospy.is_shutdown():
        throttle_percent, brake_percent = velctrl.set_vel(decoder.vel)

        send_launch.throttle_percent = throttle_percent
        send_launch.brake_percent    = brake_percent
        send_launch.steering_angle   = decoder.angle
        r.sleep()


if __name__ == '__main__':
    main()
