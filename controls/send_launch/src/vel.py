import pid

import cand


class Ctrl:
    def __init__(self, pid: pid.Controller, bus: cand.client.Bus):
        self._pid = pid
        self._bus = bus
