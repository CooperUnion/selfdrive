#!/usr/bin/env python3

import argparse
import can
import cantools
import re

from pprint import pprint
from time import sleep
from tqdm import trange

def main():
    parser = argparse.ArgumentParser(description='DBW firmware updater.')
    parser.add_argument('module_name', type=str, help="Module name (lowercase)")
    args = parser.parse_args()

    img = open(f".pio/build/{args.module_name}/firmware.bin", "rb")
    img_raw = img.read()

    db = cantools.database.load_file("../../can/igvc_can.dbc")

    updater_msg = db.get_message_by_name('dbwUpdater_Update_Data')

    trigger_msg = db.get_message_by_name('dbwUpdater_Update_Trigger')

    bus = can.interface.Bus('can0', bustype='socketcan')

    # Trigger the update
    t_data = trigger_msg.encode({'Begin': 0, 'Filler1': 0, 'Trigger': 1})
    t_msg = can.Message(arbitration_id=trigger_msg.frame_id, data=t_data, is_extended_id=False)

    response_msg = db.get_message_by_name('dbwNode_Update_Response')

    print("> Sending trigger message for ~10 seconds...")
    for i in range(0, 100):
        bus.send(t_msg)
        msg = bus.recv(timeout = 0)
        if msg is not None and msg.arbitration_id == response_msg.frame_id:
            print("> Got response!")

            sleep(0.05)
            print("> Sending update begin.")
            b_data = trigger_msg.encode({'Begin': 1, 'Filler1': 0, 'Trigger': 1})
            b_msg = can.Message(arbitration_id = trigger_msg.frame_id, data=b_data, is_extended_id=False)
            bus.send(b_msg)

            sleep(0.1)
            print("> Sending update data.")

            step = 5

            for i in trange(0, len(img_raw), step):
                raw_data = int.from_bytes(img_raw[i:i+step], 'little')
                #print(f"{img_raw[i]}, {raw_data}")
                u_data = updater_msg.encode({'Position': i/step, 'Data': raw_data})
                u_msg = can.Message(arbitration_id=updater_msg.frame_id, data=u_data, is_extended_id=False)

                bus.send(u_msg, timeout=None)
                sleep(0.00025)

#                print(img_raw[i:i+4].hex())

            done_msg = db.get_message_by_name('dbwUpdater_Update_Done')

            d_data = done_msg.encode({'FinalSize': len(img_raw)})
            d_msg = can.Message(arbitration_id=done_msg.frame_id, data=d_data, is_extended_id=False)
            bus.send(d_msg)

            print(f"> Sent {len(img_raw)} bytes!")
            print(">> DONE <<")
            return
        sleep(0.1)

    print("> Got no response :(")
    print(">> FAIL <<")


if __name__ == "__main__":
    main()
