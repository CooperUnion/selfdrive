import enum
import threading
import time

import cand


class Base(threading.Thread):
    MESSAGE_RATE_S = 0.01

    COUNTER_MAX = 256

    SYSCMD_MESSAGE_NAME       = 'dbwNode_SysCmd'
    SYSTEM_STATUS_SIGNAL_NAME = 'SystemStatus'
    COUNTER_SIGNAL_NAME       = 'Counter'

    class _sys_state(enum.Enum):
        IDLE      = 0
        UNHEALTHY = 1
        ACTIVE    = 2
        ESTOP     = 3

    def __init__(self, bus: cand.client.Bus, status_msg_name: str):
        self._bus             = bus
        self._status_msg_name = status_msg_name

        self._sys_state = self._sys_state.IDLE
        self._counter = 0

    def run():
        self._bus.send(
            self._status_msg_name,
            {
                self.SYSTEM_STATUS_SIGNAL_NAME: self._sys_state,
                self.COUNTER_SIGNAL_NAME: self._counter
            },
        )

        self._counter = (self._counter + 1) % self.COUNTER_MAX

        time.sleep(self.MESSAGE_RATE_S)
