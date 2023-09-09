#!/usr/bin/env python3

import can
import cantools
import time
from pprint import pprint
import asyncio

from cmd_can import CanListener
from node import DBWNode

estop_trigger = False

def do_estop_all(nodes, reason):
    for node in nodes:
        node.send_estop()
    print(f"\nDOING ESTOP FOR {reason}\n")
    estop_trigger = True

def main():
    pprint("DBW Test Command and Control")
    bus = can.interface.Bus('can0', bustype='socketcan')

    db = cantools.database.load_file('/selfdrive/can/igvc_can.dbc')
    pprint(db.messages)

    print("Bus set up.")

    nodes = [
        DBWNode("Encoder", 4, db, bus),
        DBWNode("Brake", 3, db, bus),
        DBWNode("Throttle", 2, db, bus),
        DBWNode("Blink", 1, db, bus),
    ]

    listener = CanListener(bus, nodes, db)

    while True:
        if estop_trigger:
            do_estop_all(nodes, "estop latched")

        for node in nodes:
            if node.get_status == "ESTOP":
                do_estop_all(nodes, "detected single node ESTOP")

        print("Checking speed...")
        pprint(listener.encoder_counts)

        if listener.time_since_last_enc_counts > 3:
            do_estop_all(nodes, "encoder timeout")

        countL, countR = listener.encoder_counts
        print(f"Left Encoder: {countL}, Right Encoder: {countR}")
        if countL > 80 or countR > 80:
            do_estop_all(nodes, f"speed violation: {countL}, {countR}")

        if abs(countL - countR):
            do_estop_all(nodes, f"encoder difference violation: {countL}, {countR}")

        for node in nodes:
            print(f"{node.name}: {node.get_status()}")

        time.sleep(0.1)
        print("")

if __name__ == "__main__":
    main()
