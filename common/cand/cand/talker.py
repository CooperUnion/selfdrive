import can
import logging
import os

from time import time
from time import time_ns

import cand.config as cfg
from cand.util import get_msg_by_name
from cand.serialization import deserialize, serialize

MAX_BUFFER_SIZE = 25
MAX_BUFFER_WAIT_TIME_SECONDS = 0.001
QUEUE_POP_TIMEOUT = 0.0001


def talker():
    log = logging.getLogger("talker")
    log.info(f"-- Started cand talker with PID {os.getpid()} --")

    time_at_last_buffer_warning = 0
    buffer_warning_suppress_count = 0

    time_at_flush = time()
    p = cfg.rdb.pipeline()

    while True:
        # Check if we need to flush the buffer
        if len(p) >= MAX_BUFFER_SIZE:
            p.execute()
            time_at_flush = time()

        elif len(p) > 0 and time() - time_at_flush > MAX_BUFFER_WAIT_TIME_SECONDS:
            p.execute()
            time_at_flush = time()

        [_, serial_msgs] = cfg.rdb.blmpop(
            QUEUE_POP_TIMEOUT,
            1,
            "queue:cansend",
            direction="LEFT",
            count=MAX_BUFFER_SIZE + 1,
        )

        if serial_msgs is None:
            continue

        if len(serial_msgs) > MAX_BUFFER_SIZE:
            if time() - time_at_last_buffer_warning > 1:
                log.critical(
                    f"Buffer is saturated ({len(serial_msgs)})! cand may not be keeping up with sent messsages."
                    + f" (suppressed {buffer_warning_suppress_count} times)"
                )
                buffer_warning_suppress_count = 0
                time_at_last_buffer_warning = time()
            else:
                buffer_warning_suppress_count += 1

        for serial_msg in serial_msgs:
            name = None
            data = None

            try:
                name, data = deserialize(serial_msg)
                msg = get_msg_by_name(name)
                packed_data = msg.encode(data)
            except Exception as e:
                log.warning(f"Failed to prepare message: {e}")
                continue

            bus_msg = can.Message(
                arbitration_id=msg.frame_id, data=packed_data, is_extended_id=False
            )

            cfg.bus.send(bus_msg)

            # python-can does not loopback messages we send by design,
            # to get around this limitation we'll need to manually set
            # the sent message in our redis database
            p.set(name, serialize((time_ns(), data)))
