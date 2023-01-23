import logging
import threading
import time

import cand


class DbwEnable(threading.Thread):
    DBW_ACTIVE_CYCLETIME_S = 0.01

    def __init__(self, *, bus: cand.client.Bus = None):
        self._bus = bus

        self._logger = logging.getLogger('DBW_Enable')

        super().__init__(daemon=True)

    def run(self):
        prv_time_ns = time.time_ns()
        dbw_active  = 0

        while True:
            time.sleep(self.DBW_ACTIVE_CYCLETIME_S)

            rec = self._bus.get('DBW_Enable')
            if rec is None: continue

            msg_time_ns, data = rec

            if msg_time_ns > prv_time_ns:
                dbw_active  = data['DBW_enable']
                prv_time_ns = msg_time_ns
                self._logger.info(f'enable: {dbw_active}')

            self._bus.send('DBW_Active', {'DBW_active': dbw_active})


class Node():
    STATUS_TIMEOUT_MS = 200
    STATUS_TIMEOUT_NS = STATUS_TIMEOUT_MS * 1_000_000

    def __init__(self, ident: str, *, bus: cand.client.Bus = None):
        self._ident  = ident
        self._bus    = bus

        self._logger = logging.getLogger(self._ident)

        self._status = 'UNDEF'
        self._status_missing = True

    def service(self):
        rec = self._bus.get(f'{self._ident}_NodeStatus')

        if rec is None:
            self._status_missing = True
            self._logger.warning('got no message!')

        else:
            timestamp, stat = rec
            time_now = time.time_ns()

            if time_now < timestamp:
                self._status_missing = True
                self._logger.critical('incoming message has newer timestamp than system!')
            elif time_now - timestamp > self.STATUS_TIMEOUT_NS:
                self._status_missing = True
                self._logger.critical(f'timeout: no message in {(time_now - timestamp) / 1_000_000} ms!')
            else:
                # we're okay
                old_status = self._status

                self._status = stat[f'{self._ident}_sysStatus']
                self._status_missing = False

                if old_status != self._status:
                    self._logger.info(f'got new status: {old_status} -> {self._status}')

    def status(self):
        return self._status

    def status_missing(self):
        return self._status_missing
