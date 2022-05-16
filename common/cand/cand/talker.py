import can
import logging
import os
import time

import cand.config as cfg
from cand.util import get_msg_by_name
from cand.serialization import deserialize

MAX_BUFFER_SIZE = 25
QUEUE_POP_TIMEOUT = 0.0001


def talker():
    log = logging.getLogger("talker")
    log.info(f"-- Started cand talker with PID {os.getpid()} --")

    time_at_last_buffer_warning = 0
    buffer_warning_suppress_count = 0

    while True:
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
            if time.time() - time_at_last_buffer_warning > 1:
                log.critical(
                    f"Buffer is saturated ({len(serial_msgs)})! cand may not be keeping up with sent messsages."
                    + f" (suppressed {buffer_warning_suppress_count} times)"
                )
                buffer_warning_suppress_count = 0
                time_at_last_buffer_warning = time.time()
            else:
                buffer_warning_suppress_count += 1

        for serial_msg in serial_msgs:
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
