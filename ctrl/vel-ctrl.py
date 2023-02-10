#!/usr/bin/env python3

import argparse
import sys
import threading
import time

import cand
import igvcutils
import numpy as np


class Ctrl:
    ENCODER_TICKS_PER_ROTATION = (4000 / 5 * 46)
    WHEEL_CIRCUMFERENCE_M      = 1.899156
    TICKS_PER_M                = WHEEL_CIRCUMFERENCE_M / ENCODER_TICKS_PER_ROTATION

    ACCEL_TO_PEDAL_SLOPE_MAPPING        = 15.4
    BRAKE_TO_PEDAL_SLOPE_MAPPING        = -58.03
    BRAKE_TO_PEDAL_SLOPE_MAPPING_OFFSET = -11.33

    def __init__(
        self,
        pid: igvcutils.ctrl.Pid,
        bus: cand.client.Bus,
        *,
        samples: int = 4,
    ):
        self._pid = pid
        self._bus = bus

        self._samples      = samples
        self._vel_prv_time = 0
        self._vel_hist     = np.zeros(self._samples)
        self._vel_index    = 0
        self._vel_filtered = 0.0

        self._bus_thread = threading.Thread(
            target=self._vel_filter,
            daemon=True,
        )
        self._bus_thread.start()

    def _accel2pedal(self, accel: float) -> float:
        return self.ACCEL_TO_PEDAL_MAPPING * accel

    def _brake2pedal(self, accel: float) -> float:
        return (self.BRAKE_TO_PEDAL_MAPPING * accel) + self.BRAKE_TO_PEDAL_MAPPING_OFFSET

    def _delta_ticks(self):
        pass

    def _pedal_ctrl(self, vel_act: float, vel_des: float, accel_des: float) -> tuple[float, float]:
        # TODO: make this clean
        if vel_des == 0.0:
            return 0, 50

        if vel_act < 0:
            if vel_act > -0.5:
                if accel_des > 0 and vel_des > 0:
                    return self._accel2pedal(accel_des), 0  # accounts for rolling backwards on an incline when the desired motion is forward

                else:
                    return 0, self._brake2pedal(accel_des)  # accounts for rolling backwards when the desired motion is to stop in order to reverse
            else:
                return 0, 50                                # accounts for a non-float input-- just brakes

        elif vel_act >= 0:
            if accel_des > 0:
                return self._accel2pedal(accel_des), 0      # accounts for when the car is moving forward and the desired motion is to increase speed

            elif accel_des <= 0 :
                return 0, self._brake2pedal(accel_des)      # accounts for the the car is moving forward and the desired motion is to slow down or stop

        else:
            return 0, 50                                    # accounts for a non-float input -- just brakes

    def _tick2vel(self, ticks: int, time: int) -> float:
        return self.TICKS_PER_M * ticks / time

    def _vel_filter(self):
        while True:
            reading = self._bus.get('CTRL_EncoderData')
            if not reading: continue

            if self._vel_prv_time == reading[0]: continue
            self._vel_prv_time = reading[0]

            self._vel_hist[self._vel_index] = self._tick2vel(
                (reading[1]['encoderLeft'] + reading[1]['encoderRight']) / 2,
                0.01
            )

            self._vel_index = (self._vel_index + 1) % self._samples

            self._vel_filtered = np.mean(self._vel_hist)

    def set_vel(self, target_vel: float) -> tuple[float, float]:
        actual_vel = self._vel_filtered

        accel_des = self._pid.step(target_vel, actual_vel)

        return self._pedal_ctrl(actual_vel, target_vel, accel_des)


def main():
    argparser = argparse.ArgumentParser(description='velocity controller')

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

    args = argparser.parse_args()
    args.rate = abs(args.rate)

    ts = 1 / args.rate

    bus = cand.client.Bus()

    pid = Ctrl(
        igvcutils.ctrl.Pid(
            kp=args.kp,
            ki=args.ki,
            kd=args.kd,
            ts=ts,
        ),
        bus=bus,
    )

    while True:
        lin_vel = 0

        rec = bus.get('DBW_VelocityCommand')
        if rec:
            unix_time_ns, data = rec

            if time.time_ns() - unix_time_ns <= 200_000_000:
                lin_vel = data['linearVelocity']

        throttle_percent, brake_percent = pid.set_vel(lin_vel)

        print(f'brake_percent: {brake_percent} throttle_percent: {throttle_percent}')

        bus.send(
            'DBW_RawVelocityCommand',
            {
                'brakePercent':    brake_percent,
                'throttlePercent': throttle_percent,
            },
        )

        time.sleep(ts)


if __name__ == '__main__':
    main()
