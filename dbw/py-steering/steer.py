import math
import threading
import time

import cand
import igvcutils

import base


class Steer(threading.Thread):
    MESSAGE_RATE_S     = 0.01
    ABS_ENC_TIMEOUT_NS = 20_000_000

    ENCODER_TO_ANGLE_SLOPE_MAPPING           = 0.0029
    ENCODER_TO_ANGLE_SLOPE_MAPPING_INTERCEPT = 0.0446

    STEERING_DATA_MESSAGE_NAME    = 'dbwNode_Steering_Data'
    STEERING_ANGLE_SIGNAL_NAME    = 'Angle'
    STEERING_ABS_ENC_MESSAGE_NAME = 'steering_Absolute_Encoder'
    STEERING_POS_SIGNAL_NAME      = 'Pos'

    def __init__(
        self,
        bus:  cand.client.Bus,
        pid:  igvcutils.ctrl.Pid,
        base: base.Base
    ):
        self._bus  = bus
        self._pid  = pid
        self._base = base

        self._cur_angle = 0.0
        self._prv_enc_unix_time_ns = None

        threading.Thread.__init__(self, daemon=True)

    def _enc2angle(self, val: int) -> float:
        return (self.ENCODER_TO_ANGLE_SLOPE_MAPPING * val) + self.ENCODER_TO_ANGLE_SLOPE_MAPPING_INTERCEPT

    def run(self):
        while True:
            rec = self._bus.get(self.STEERING_ABS_ENC_MESSAGE_NAME)

            if rec:
                unix_time_ns, data = rec

                if time.time_ns() - unix_time_ns >= self.ABS_ENC_TIMEOUT_NS:
                    self._base.set_state_estop()

                # the absolute encoder data is currently not decoded
                # correctly so we'll need to re-encode the data
                data[self.STEERING_POS_SIGNAL_NAME] = igvcutils.can.endianswap(
                    data[self.STEERING_POS_SIGNAL_NAME],
                    'little',
                    dst_signed=True,
                )

                self._cur_angle = self._enc2angle(
                    data[self.STEERING_POS_SIGNAL_NAME],
                )

            elif self._prv_enc_unix_time_ns:
                if time.time_ns() - self._prv_enc_unix_time_ns >= self.ABS_ENC_TIMEOUT_NS:
                    self._base.set_state_estop()

            else:
                self._prv_enc_unix_time_ns = time.time_ns()

            self._bus.send(
                self.STEERING_DATA_MESSAGE_NAME,
                {
                    self.STEERING_ANGLE_SIGNAL_NAME: math.radians(self._cur_angle),
                },
            )

            time.sleep(self.MESSAGE_RATE_S)
