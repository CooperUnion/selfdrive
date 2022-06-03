import threading

import cand
import numpy as np

import pid


class Ctrl:
    ENCODER_TICKS_PER_ROTATION = 4000
    WHEEL_CIRCUMFRANCE_M       = 1.899156
    TICKS_PER_M                = WHEEL_CIRCUMFRANCE_M / ENCODER_TICKS_PER_ROTATION

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
        return self.ACCEL_TO_PEDAL_MAPPING * accel

    def _brake2pedal(self, accel: float) -> float:
        return (self.BRAKE_TO_PEDAL_MAPPING * accel) + self.BRAKE_TO_PEDAL_MAPPING_OFFSET

    def _tick2vel(self, ticks: int, time: int) -> float:
        return self.TICKS_PER_M * ticks / time

    def _vel_filter(self):
        while True:
            reading = self._bus.get('dbwNode_Encoder_Data')
            if not reading: continue

            if self._vel_prv_time == reading[0]: continue
            self._vel_prv_time = reading[0]

            self._vel_hist[self._vel_index] = self._tick2vel(
                (reading[1]['Encoder0'] + reading[1]['Encoder1']) / 2,
                reading[1]['Time'] / 1_000_000,
            )

            self._vel_index = (self._vel_index + 1) % self._samples

            self._vel_filtered = np.mean(self._vel_hist)

    def set_vel(self, vel: float):
        vel_set = self._pid.step(vel, self._vel_filtered)

        brake = self._brake2pedal(vel_set)
        accel = self._accel2pedal(vel_set)

        # until we get F/N/R working
        if vel <= 0.0: accel = 0.0

        self._bus.send(
            'dbwNode_Accel_Cmd',
            {'ThrottleCmd': min(abs(accel), 100) / 100, 'ModeCtrl': 1},
        )
        self._bus.send(
            'dbwNode_Brake_Cmd',
            {'BrakeCmd': min(abs(brake), 100) / 100},
        )
