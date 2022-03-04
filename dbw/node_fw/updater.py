#!/bin/bash

import can
import cantools

from pprint import pprint

from time import sleep

from tqdm import trange

def main():
    img = open(".pio/build/igvc_dbw_node/firmware.bin", "rb")
    img_raw = img.read()

    db = cantools.database.load_file("../../can/igvc_can.dbc")
    print(db.messages)

    print('')

    updater_msg = db.get_message_by_name('dbwUpdater_Update_Data')
    pprint(updater_msg.signals)

    print('')

    trigger_msg = db.get_message_by_name('dbwUpdater_Update_Trigger')
    pprint(trigger_msg.signals)

    print('')

    bus = can.interface.Bus('can0', bustype='socketcan')

    # Trigger the update
    t_data = trigger_msg.encode({'Begin': 0, 'Filler1': 0, 'Trigger': 1})
    t_msg = can.Message(arbitration_id=trigger_msg.frame_id, data=t_data, is_extended_id=False)

    bus.send(t_msg)

    response_msg = db.get_message_by_name('dbwNode_Update_Response')
    pprint(response_msg.signals)

    print("Sent trigger message; waiting for node response...")
    for msg in bus:
        if True: 
#        if msg.arbitration_id == response_msg.frame_id:
            print("Got response!")
            pprint(msg)

            sleep(0.05)
            print("Sending update begin.")
            b_data = trigger_msg.encode({'Begin': 1, 'Filler1': 0, 'Trigger': 1})
            b_msg = can.Message(arbitration_id = trigger_msg.frame_id, data=b_data, is_extended_id=False)
            bus.send(b_msg)

            sleep(0.1)
            print("Sending update data.")

            step = 5

            for i in trange(0, len(img_raw), step):
            #    sleep(0.05)
                
                raw_data = int.from_bytes(img_raw[i:i+step], 'little')
                #print(f"{img_raw[i]}, {raw_data}")
                u_data = updater_msg.encode({'Position': i/step, 'Data': raw_data})
                u_msg = can.Message(arbitration_id=updater_msg.frame_id, data=u_data, is_extended_id=False)

                bus.send(u_msg, timeout=None)
                sleep(0.00025)

#                print(img_raw[i:i+4].hex())

            done_msg = db.get_message_by_name('dbwUpdater_Update_Done')
            pprint(done_msg.signals)

            d_data = done_msg.encode({'FinalSize': len(img_raw)})
            d_msg = can.Message(arbitration_id=done_msg.frame_id, data=d_data, is_extended_id=False)
            bus.send(d_msg)

            return


if __name__ == "__main__":
    main()
