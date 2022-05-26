import threading

import cand
import numpy as np

import pid


class Ctrl:
    ENCODER_TICKS_PER_ROTATION = 4000
    WHEEL_CIRCUMFRANCE_M       = 1.899156
    TICKS_PER_M                = WHEEL_CIRCUMFRANCE_M / ENCODER_TICKS_PER_ROTATION

    ACCEL_TO_PEDAL_SLOPE_MAPPING           = 15.4
    BRAKE_TO_PEDAL_SLOPE_MAPPING           = -58.03
    BRAKE_TO_PEDAL_SLOPE_MAPPING_INTERCEPT = -11.33

    ENCODER_CAN_MESSAGE_NAME  = 'dbwNode_Encoder_Data'
    ENOCDER0_SIGNAL_NAME      = 'Encoder0'
    ENOCDER1_SIGNAL_NAME      = 'Encoder1'
    TIME_DETLA_US_SIGNAL_NAME = 'Time'

    def __init__(
        self,
        pid: pid.Controller,
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
        return self.ACCEL_TO_PEDAL_SLOPE_MAPPING * accel

    def _brake2pedal(self, accel: float) -> float:
        return (self.BRAKE_TO_PEDAL_SLOPE_MAPPING * accel) + self.BRAKE_TO_PEDAL_SLOPE_MAPPING_INTERCEPT

    def _tick2vel(self, ticks: int, time: int) -> float:
        return self.TICKS_PER_M * ticks / time

    def _vel_filter(self):
        while True:
            reading = self._bus.get(self.ENCODER_CAN_MESSAGE_NAME)
            if not reading: continue

            if self._vel_prv_time == reading[0]: continue
            self._vel_prv_time = reading[0]

            self._vel_hist[self._vel_index] = self._tick2vel(
                (reading[1][self.ENOCDER0_SIGNAL_NAME] + reading[1][self.ENOCDER1_SIGNAL_NAME]) / 2,
                reading[1][self.TIME_DETLA_US_SIGNAL_NAME],
            )

            self._vel_index = (self._vel_index + 1) % self._samples

            self._vel_filtered = np.mean(self._vel_hist)
