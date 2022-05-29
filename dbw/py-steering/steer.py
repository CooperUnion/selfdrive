import math
import threading
import time

import cand
import igvcutils
import odrive
import odrive.enums
import odrive.utils

import base


class Steer(threading.Thread):
    MESSAGE_RATE_S        = 0.01
    ABS_ENC_TIMEOUT_NS    = 20_000_000
    CMD_TIMEOUT_NS        = 200_000_000
    ODRIVE_INIT_TIMEOUT_S = 0.5

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

        self._od   = odrive.find_any(timeout=self.ODRIVE_INIT_TIMEOUT_S)
        self._axis = self._od.axis0 if self._od else None

        if self._od:
            self._od.clear_errors()

            self._axis.requested_state = odrive.enums.AXIS_STATE_FULL_CALIBRATION_SEQUENCE

            while self._axis.current_state != odrive.enums.AXIS_STATE_IDLE:
                pass

            self._axis.requested_state                = odrive.enums.AXIS_STATE_CLOSED_LOOP_CONTROL
            self._axis.controller.config.input_mode   = odrive.enums.INPUT_MODE_PASSTHROUGH
            self._axis.controller.config.control_mode = odrive.enums.CONTROL_MODE_VELOCITY_CONTROL


        self._encoder_timeout   = 0
        self._odrive_connection = 1 if self._od else 0

        self._cur_angle = 0.0
        self._des_angle = 0.0
        self._prv_enc_unix_time_ns = None

        threading.Thread.__init__(self, daemon=True)

    def _enc2angle(self, val: int) -> float:
        return (self.ENCODER_TO_ANGLE_SLOPE_MAPPING * val) + self.ENCODER_TO_ANGLE_SLOPE_MAPPING_INTERCEPT

    def run(self):
        while True:
            rec = self._bus.get('steering_Absolute_Encoder')

            if rec:
                unix_time_ns, data = rec

                if time.time_ns() - unix_time_ns >= self.ABS_ENC_TIMEOUT_NS:
                    self._encoder_timeout = 1
                else:
                    self._encoder_timeout = 0

                # the absolute encoder data is currently not decoded
                # correctly so we'll need to re-encode the data
                data['Pos'] = igvcutils.can.endianswap(
                    data['Pos'],
                    'little',
                    dst_signed=True,
                )

                self._cur_angle = self._enc2angle(data['Pos'])

            elif self._prv_enc_unix_time_ns:
                if time.time_ns() - self._prv_enc_unix_time_ns >= self.ABS_ENC_TIMEOUT_NS:
                    self._encoder_timeout = 1

            else:
                self._prv_enc_unix_time_ns = time.time_ns()

            self._bus.send(
                'dbwNode_Steering_Data',
                {
                    'Angle':             math.radians(self._cur_angle),
                    'EncoderTimeoutSet': self._encoder_timeout,
                    'ODriveConnected':   self._odrive_connection,
                },
            )

            if self._base.dbw_currently_active():
                if self._encoder_timeout or not self._odrive_connection:
                    self._base.set_state_estop()
                    time.sleep(self.MESSAGE_RATE_S)
                    continue

                rec = self._bus.get('dbwNode_Steering_Cmd')

                if rec:
                    unix_time_ns, data = rec

                    if time.time_ns() - unix_time_ns >= self.CMD_TIMEOUT_NS:
                        self._base.set_state_estop()
                        time.sleep(self.MESSAGE_RATE_S)
                        continue

                    self._des_angle = math.degrees(data['Angle'])

                elif self._prv_cmd_unix_time_ns:
                    if time.time_ns() - self._prv_cmd_unix_time_ns >= self.CMD_TIMEOUT_NS:
                        self._sys_state = self._sys_states.ESTOP
                        time.sleep(self.MESSAGE_RATE_S)
                        continue

                else:
                    self._prv_cmd_unix_time_ns = time.time_ns()
                    time.sleep(self.MESSAGE_RATE_S)
                    continue

            elif self._od:
                # disable odrive here
                pass

            time.sleep(self.MESSAGE_RATE_S)
