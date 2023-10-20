from base64 import encode
from os import EX_CANTCREAT
from threading import Thread

from pprint import pprint

from node import DBWNode

import time


class CanListener(Thread):
    def __init__(self, bus, nodes: list[DBWNode], db):
        self.bus = bus
        self.nodes = nodes
        self.db = db
        self.encoder_counts = [0, 0]
        self.time_at_last_enc_counts = 0
        self.time_since_last_enc_counts = 0

        super().__init__(target=self.listener)
        self.daemon = True
        self.start()

        pprint("Created CAN listener.")

    def listener(self):
        encoder_msg = self.db.get_message_by_name('dbwNode_Encoder_Data')
        self.time_at_last_enc_counts = time.time()
        while True:
            try:
                for msg in self.bus:
                    for node in self.nodes:
                        if node.status_msg.frame_id == msg.arbitration_id:
                            status = self.db.decode_message(
                                msg.arbitration_id, msg.data
                            )['SystemStatus']
                            node.set_status(status)

                    time_now = time.time()
                    self.time_since_last_enc_counts = (
                        time_now - self.time_at_last_enc_counts
                    )
                    # print(self.time_since_last_enc_counts)

                    if encoder_msg.frame_id == msg.arbitration_id:
                        self.encoder_counts = self.db.decode_message(
                            msg.arbitration_id, msg.data
                        )
                        self.time_at_last_enc_counts = time_now

            except Exception as e:
                print("CAN Listener error!")
                print(e)
