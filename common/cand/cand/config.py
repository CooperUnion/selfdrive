# Global configuration

import can
import cantools
import redis


def init(_bus: can.BusABC, _dbc: cantools.database, _rdb: redis.Redis):
    global bus, dbc, rdb

    bus = _bus
    dbc = _dbc
    rdb = _rdb
