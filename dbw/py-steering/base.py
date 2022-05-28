import enum
import threading

import cand


class Base(threading.Thread):
    class _sys_state(enum.Enum):
        IDLE      = 0
        UNHEALTHY = 1
        ACTIVE    = 2
        ESTOP     = 3

    def __init__(self, bus: cand.client.Bus, mod_ident: int):
        self._bus       = bus
        self._mod_ident = mod_ident

    def run():
        pass
