import logging

import cand


class Unit:
    def __init__(self, *, bus: cand.client.Bus = None):
        self._bus = bus

        self._logger = logging.getLogger(type(self).__name__)

    @property
    def bus(self) -> cand.client.Bus:
        return self._bus

    @bus.setter
    def bus(self, val: cand.client.Bus):
        self._bus = val

    @bus.deleter
    def bus(self):
        del self._bus

    def abort(self, reason: str = 'FAIL') -> bool:
        self._logger.error(f'aborted for \'{reason}\'')
        self._bus.send('dbwESTOP', {'Source': 'SAFED', 'Reason': reason})
        return False

    def test(self):
        pass
