import enum
import threading
import time

import cand


class Base(threading.Thread):
    MESSAGE_RATE_S        = 0.01
    DBW_ACTIVE_TIMEOUT_NS = 20_000_000

    COUNTER_MAX = 256

    NODE_STATUS_MESSGAE_NAME  = 'dbwNode_Status'
    SYSTEM_STATUS_SIGNAL_NAME = 'SystemStatus'
    COUNTER_SIGNAL_NAME       = 'Counter'
    SYSCMD_MESSAGE_NAME       = 'dbwNode_SysCmd'
    DBWACTIVE_SIGNAL_NAME     = 'DbwActive'
    ESTOP_SIGNAL_NAME         = 'ESTOP'

    class _sys_states(enum.IntEnum):
        IDLE      = 0
        UNHEALTHY = 1
        ACTIVE    = 2
        ESTOP     = 3

    def __init__(self, bus: cand.client.Bus, mod_ident: str):
        self._bus       = bus
        self._mod_ident = mod_ident

        self._sys_state = self._sys_states.IDLE
        self._counter = 0

        threading.Thread.__init__(self, daemon=True)

    def run(self):
        while True:
            rec = self._bus.get(self.SYSCMD_MESSAGE_NAME)

            if rec:
                unix_time_ns, data = rec

                if data[self.DBWACTIVE_SIGNAL_NAME] and \
                    self._sys_state == self._sys_states.IDLE:
                    self._sys_state = self._sys_states.ACTIVE

                elif not data[self.DBWACTIVE_SIGNAL_NAME] and \
                    self._sys_state == self._sys_states.ACTIVE:
                    self._sys_state = self._sys_states.IDLE

                if data[self.ESTOP_SIGNAL_NAME]:
                    self._sys_state = self._sys_states.ESTOP

                if self._sys_state == self._sys_states.ACTIVE and \
                    time.time_ns() - unix_time_ns >= self.DBW_ACTIVE_TIMEOUT_NS:
                    self._sys_state = self._sys_states.IDLE

            self._bus.send(
                self.NODE_STATUS_MESSGAE_NAME + '_' + self._mod_ident,
                {
                    self.SYSTEM_STATUS_SIGNAL_NAME: self._sys_state,
                    self.COUNTER_SIGNAL_NAME: self._counter
                },
            )

            self._counter = (self._counter + 1) % self.COUNTER_MAX

            time.sleep(self.MESSAGE_RATE_S)

    def set_state_estop(self):
        self._sys_state = self._sys_states.ESTOP

    def dbw_currently_active(self) -> bool:
        return self._sys_state == self._sys_states.ACTIVE
