import cantools

from functools import cache

import cand.config as cfg


@cache
def get_msg_by_name(name: str) -> cantools.database.can.message.Message:
    return cfg.dbc.get_message_by_name(name)


@cache
def get_msg_by_frameid(name: str) -> cantools.database.can.message.Message:
    try:
        return cfg.dbc.get_message_by_frame_id(name)
    except KeyError:
        return None
