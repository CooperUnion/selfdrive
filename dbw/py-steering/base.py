import logging
import threading
import time

import cand


class Base(threading.Thread):
    MESSAGE_RATE_S        = 0.01
    DBW_ACTIVE_TIMEOUT_NS = 200_000_000

    COUNTER_MAX = 256

    def __init__(self, bus: cand.client.Bus, mod_ident: str):
        self._bus       = bus
        self._mod_ident = mod_ident

        self._logger = logging.getLogger('base')

        self._sys_state = 'IDLE'
        self._counter = 0

        self._init_time_ns = time.time_ns()

        threading.Thread.__init__(self, daemon=True)

    def run(self):
        while True:
            rec = self._bus.get('DBW_Active')

            if rec and self._sys_state != 'ESTOP':
                msg_time, data = rec

                cur_time = time.time_ns()

                if \
                    data['active'] and \
                    cur_time - msg_time < self.DBW_ACTIVE_TIMEOUT_NS and \
                    self._sys_state == 'IDLE':

                    if self._sys_state != 'ACTIVE':
                        self._logger.info('state: ACTIVE')

                    self._sys_state = 'ACTIVE'

                elif \
                    not data['active'] or\
                    cur_time - msg_time >= self.DBW_ACTIVE_TIMEOUT_NS and \
                    self._sys_state == 'ACTIVE':

                    if self._sys_state != 'IDLE':
                        self._logger.info('state: IDLE')

                    self._sys_state = 'IDLE'

            rec = self._bus.get('DBW_ESTOP')

            if rec:
                msg_time, data = rec

                cur_time = time.time_ns()

                if msg_time >= self._init_time_ns:
                    if self._sys_state != 'ESTOP':
                        self._logger.critical('state: ESTOP')

                    self._sys_state = 'ESTOP'

            self._bus.send(
                'NodeStatus_' + self._mod_ident,
                {
                    'sysStatus':         self._sys_state,
                    'counter':              self._counter,
                    'resetReason':          'UNKNOWN',
                    'esp32ResetReasonCode': 0,
                },
            )

            self._counter = (self._counter + 1) % self.COUNTER_MAX

            time.sleep(self.MESSAGE_RATE_S)

    def set_state_estop(self, reason: str, err_msg: str = None):
        self._sys_state = 'ESTOP'
        self._bus.send(
            'DBW_ESTOP',
            {'src': 'NODE', 'reason': reason},
        )
        self._logger.warn(f'ESTOP reason: {reason}')
        if err_msg: self._logger.error(err_msg)
        self._logger.critical('state: ESTOP')

    def dbw_currently_active(self) -> bool:
        return self._sys_state == 'ACTIVE'
