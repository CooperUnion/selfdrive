import threading

import cand
import igvcutils

import base


class Steer(threading.Thread):
    ENCODER_TO_ANGLE_SLOPE_MAPPING           = 0.0029
    ENCODER_TO_ANGLE_SLOPE_MAPPING_INTERCEPT = 0.0446

    def __init__(
        self,
        bus:  cand.client.Bus,
        pid:  igvcutils.ctrl.Pid,
        base: base.Base
    ):
        self._bus  = bus
        self._pid  = pid
        self._base = base

        threading.Thread.__init__(self, daemon=True)

    def _enc2angle(self, val: int) -> float:
        return (self.ENCODER_TO_ANGLE_SLOPE_MAPPING * val) + self.ENCODER_TO_ANGLE_SLOPE_MAPPING_INTERCEPT
