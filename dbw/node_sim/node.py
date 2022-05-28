import logging
import time

from cand.client import Bus


class Node():
    def __init__(self, ident: str):
        self._bus = Bus(redis_port=6378)
        self._ident = ident
        self._system_state = 'SYS_STATE_IDLE'


    def _check_messages(self):
        _, cmd = self._bus.get('dbwNode_SysCmd')
        # the core logic here is in node_fw base/base.c.

        if cmd['DbwActive'] == 1 and self._system_state == 'SYS_STATE_IDLE':
            self._system_state = 'SYS_STATE_DBW_ACTIVE'
        elif cmd['DbwActive'] != 1 and self._system_state == 'SYS_STATE_DBW_ACTIVE':
            self._system_state = 'SYS_STATE_IDLE'

        if cmd['ESTOP'] != 0:
            self._system_state = 'SYS_STATE_ESTOP'


    def _send_messages(self):
        state_to_report = {
            'SYS_STATE_IDLE': 'IDLE',
            'SYS_STATE_DBW_ACTIVE': 'ACTIVE',
            'SYS_STATE_ESTOP': 'ESTOP'
        }

        report = state_to_report.get(self._system_state)
        if report is None:
            report = 'UNHEALTHY'


        self._bus.send(f'dbwNode_Status_{self._ident}', {'SystemStatus': report, 'Counter': 0})


    def _run(self):
        while True:
            self._check_messages()
            self._send_messages()
            time.sleep(0.1)


    def reset(self):
        self._system_state = 'SYS_STATE_IDLE'

    # all other functions should be "administrative"
