import threading

import cand
import igvcutils


class Steer(threading.Thread):
    def __init__(self, bus: cand.client.Bus, pid: igvcutils.ctrl.Pid):
        self._bus = bus
        self._pid = pid

        threading.Thread.__init__(self, daemon=True)
