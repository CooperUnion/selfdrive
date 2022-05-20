# Welcome to cand.

import logging
import os

from time import time
from time import time_ns

import cand.config as cfg

from cand.util import get_msg_by_frameid
from cand.serialization import serialize

MAX_BUFFER_WAIT_TIME_SECONDS = 0.001
MAX_BUFFER_SIZE = 10
CAN_RECV_TIMEOUT_SECONDS = 0.005


def cand():
    log = logging.getLogger("listener")
    log.info(f"-- Started cand listener with PID {os.getpid()} --")

    p = cfg.rdb.pipeline()  # Redis buffer ("pipeline")
    time_at_flush = time()

    log.info("Starting listen loop.")
    while True:
        # Check if we need to flush the buffer
        if len(p) >= MAX_BUFFER_SIZE:
            p.execute()
            time_at_flush = time()

        elif len(p) > 0 and time() - time_at_flush > MAX_BUFFER_WAIT_TIME_SECONDS:
            p.execute()
            time_at_flush = time()

        # Poll the bus for a message.
        #
        # Note: Right now, we're just waiting for the timeout period.
        # If cand gains more functionality where it has to service things
        # other than the bus in this loop, it would be smart to use the
        # asyncio recv and do other stuff for a little while.
        #
        # This timeout is also currently the only thing rate-limiting this
        # loop. We don't want to sleep!
        raw_msg = cfg.bus.recv(timeout=CAN_RECV_TIMEOUT_SECONDS)
        if raw_msg is None:
            continue

        # Find the dbc message
        frame_id = raw_msg.arbitration_id

        msg = get_msg_by_frameid(frame_id)
        if msg is None:  # unrecognized frame ID
            log.debug(f"Unrecognized message: {' '.join(str(raw_msg).split())}")
            continue

        # Serialize and post the message data
        data = (time_ns(), msg.decode(raw_msg.data))
        serialized_data = serialize(data)

        p.set(msg.name, serialized_data)  # commit to Redis buffer!
