import logging
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

    ENCODER_TO_ANGLE_SLOPE        = 0.0029
    ENCODER_TO_ANGLE_SLOPE_OFFSET = 0.0446

    def __init__(
        self,
        bus:  cand.client.Bus,
        pid:  igvcutils.ctrl.Pid,
        base: base.Base
    ):
        self._bus  = bus
        self._pid  = pid
        self._base = base

        self._logger = logging.getLogger('steer')

        self._od   = None
        self._axis = None

        self._odrive_connection = None

        try:
            self._logger.info('attempting connection to ODrive')
            self._od = odrive.find_any(timeout=self.ODRIVE_INIT_TIMEOUT_S)
            self._axis = self._od.axis0

            self._logger.info('ODrive connected')
            self._odrive_connection = 1

            if not self._axis.motor.is_calibrated or \
                self._axis.error or \
                self._axis.motor.error or \
                self._axis.sensorless_estimator.error or \
                self._axis.encoder.error or \
                self._axis.controller.error:

                self._od.clear_errors()

                self._logger.info('calibrating odrive')
                self._axis.requested_state = odrive.enums.AXIS_STATE_FULL_CALIBRATION_SEQUENCE
                while self._axis.current_state != odrive.enums.AXIS_STATE_IDLE:
                    pass

        except TimeoutError:
            self._logger.critical('could not connect to ODrive')
            self._odrive_connection = 0


        self._encoder_timeout = 0

        self._odrive_enabled = False

        self._cur_angle = 0.0
        self._des_angle = 0.0
        self._prv_enc_unix_time_ns = None
        self._prv_cmd_unix_time_ns = None

        threading.Thread.__init__(self, daemon=True)

    def _odrive_en(self, yes: bool):
        if yes and not self._odrive_enabled:
            self._axis.requested_state                = odrive.enums.AXIS_STATE_CLOSED_LOOP_CONTROL
            self._axis.controller.config.input_mode   = odrive.enums.INPUT_MODE_PASSTHROUGH
            self._axis.controller.config.control_mode = odrive.enums.CONTROL_MODE_VELOCITY_CONTROL
            self._odrive_enabled = True

            self._logger.info('ODrive enabled')

        elif not yes and self._odrive_enabled:
            self._axis.requested_state      = odrive.enums.AXIS_STATE_IDLE
            self._axis.controller.input_vel = 0
            self._odrive_enabled = False

            self._logger.info('ODrive disabled')

    def _enc2angle(self, val: int) -> float:
        return (self.ENCODER_TO_ANGLE_MAPPING * val) + self.ENCODER_TO_ANGLE_MAPPING_OFFSET

    def disable_odrive(self):
        if self._od: return self._odrive_en(False)

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
                    self._logger.critical('ESTOP: hardware not ready')
                    self._base.set_state_estop()
                    time.sleep(self.MESSAGE_RATE_S)
                    continue

                rec = self._bus.get('dbwNode_Steering_Cmd')

                if rec:
                    unix_time_ns, data = rec

                    if time.time_ns() - unix_time_ns >= self.CMD_TIMEOUT_NS:
                        self._logger.critical('ESTOP: command timeout')
                        self._base.set_state_estop()
                        time.sleep(self.MESSAGE_RATE_S)
                        continue

                    self._des_angle = math.degrees(data['Angle'])

                    self._odrive_en(True)
                    self._axis.controller.input_vel = self._pid.step(
                        self._des_angle,
                        self._cur_angle,
                    )

                elif self._prv_cmd_unix_time_ns:
                    if time.time_ns() - self._prv_cmd_unix_time_ns >= self.CMD_TIMEOUT_NS:
                        self._logger.critical('ESTOP: command timeout')
                        self._base.set_state_estop()
                        time.sleep(self.MESSAGE_RATE_S)
                        continue

                else:
                    self._prv_cmd_unix_time_ns = time.time_ns()
                    time.sleep(self.MESSAGE_RATE_S)
                    continue

            elif self._od:
                self._odrive_en(False)

            time.sleep(self.MESSAGE_RATE_S)
