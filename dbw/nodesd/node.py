import logging
import time

from cand.client import Bus


MAX_STATUS_TIME_DELTA_MS = 200
MAX_STATUS_TIME_DELTA_NS = MAX_STATUS_TIME_DELTA_MS * 1000000


class Node():
    def __init__(self, ident: str):
        self._log = logging.getLogger(f'node_{ident}')
        self._bus = Bus()
        self._ident = ident
        self._status = 'UNDEF'
        self._status_missing = True

    def service(self):
        res = self._bus.get(f'dbwNode_Status_{self._ident}')

        if res is None:
            self._status_missing = True
            self._log.warning("Got no message!")
        else:
            timestamp, stat = res
            time_now = time.time_ns()

            if time_now < timestamp:
                self._status_missing = True
                self._log.critical("Incoming message has newer timestamp than system!")
            elif time_now - timestamp > MAX_STATUS_TIME_DELTA_NS:
                self._status_missing = True
                self._log.critical(f"Timeout, no message in {(time_now - timestamp) / 1000000} ms!")
            else:
                # we're okay
                old_status = self._status

                self._status = stat['SystemStatus']
                self._status_missing = False

                if old_status != self._status:
                    self._log.info(f"Got new status: {old_status} -> {self._status}")

    def status(self):
        return self._status

    def status_missing(self):
        return self._status_missing
