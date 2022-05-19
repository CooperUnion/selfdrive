#!/usr/bin/env python3

import argparse
import time
import sys

import cand


class Updater:
    MAGIC_PACKET_TIMEOUT_NS = 8_000_000_000
    MAGIC_PACKET_SLEEP_S    = 0.4

    def __init__(self, bus=cand.client.Bus()):
        self.bus = bus

    def update(self, mod_name: str, bin_path: str) -> int:
        bin = None
        with open(bin_path, 'rb') as bin_fp:
            bin = bin_fp.read()

        _mod_name = mod_name.capitalize()
        binsiz    = len(bin)

        print(f"starting update for node '{mod_name}'")
        print(f'waiting for response...', end='', flush=True)

        update_init_time = time.time_ns()
        while True:
            self.bus.send(f'dbwBL_Magic_Packet_{_mod_name}', {'Size': binsiz})
            time.sleep(self.MAGIC_PACKET_SLEEP_S)

            msg = self.bus.get(f'dbwBL_Metadata_{_mod_name}')

            if msg:
                msg_time = msg[0]
                msg_data = msg[1]

                if msg_time < update_init_time:
                    if time.time_ns() - update_init_time > self.MAGIC_PACKET_TIMEOUT_NS:
                        print(' FAIL')
                        return -1
                    else:
                        continue

                if msg_data['Ready']:
                    print(' OK')
                    break
                else:
                    print(' FAIL')
                    return -1

            elif time.time_ns() - update_init_time > self.MAGIC_PACKET_TIMEOUT_NS:
                print(' FAIL')
                return -1


def main():
    argparser = argparse.ArgumentParser(description='node_fw updater')

    argparser.add_argument(
        '-m',
        '--module',
        help='module to flash',
    )

    args = argparser.parse_args()

    updater = Updater()


if __name__ == '__main__':
    main()
