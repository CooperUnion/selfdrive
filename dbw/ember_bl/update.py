# spdx-license-identifier: MPL-2.0
# ember_bl updater.
# (c) 2023 Daniel Mezhiborsky

import argparse
import can
import coloredlogs
import datetime
import isotp
import logging

from cand.client import Bus
from threading import Thread
from time import sleep
from tqdm import trange

CHUNK_SIZE = 4095
iface = 'can0'

SLEEP_WAIT = 0.01

TARGET_NODE = 'THROTTLE'

# current chunk; global and shared
chunk = 0

class TargetNode():
    def __init__(self, node: str, total_size: int):
        self.bus = Bus()
        self.log = logging.getLogger(f"target_node_{node}")

        self.node = node
        self.total_size = total_size

        self.stop_set = False

        self.update_control_thread = Thread(target = self.update_control, daemon = True)
        self.update_control_thread.start()
        self.log.info("Started UpdateControl thread.")

    def stop(self) -> None:
        self.stop_set = True

    def update_control(self) -> None:
        while True:
            if self.stop_set:
                exit(0)

            signals = {
                'UPD_updateSizeBytes': self.total_size,
                'UPD_currentIsoTpChunk': chunk
            }

            self.bus.send(f'UPD_UpdateControl_{self.node}', signals)
            sleep(0.02)

    def state(self) -> str:
        message = f'{self.node}BL_Status'
        data = self.bus.get_data(message)
        if data is None:
            self.log.error(f"Missing {message} from cand... is the node present?")
            exit(-1)

        return data[f'{self.node}BL_state']

    def is_alive(self) -> bool:
        message = f'{self.node}BL_Status'
        dt = self.bus.get_time_delta(message)

        if dt is None:
            return False

        ALIVE_DELTA_NS = 500 * 1000 * 1000 # 0.5 seconds
        return dt <= ALIVE_DELTA_NS


def main():
    parser = argparse.ArgumentParser(description='Update a node over CAN.')
    parser.add_argument('--iface', type=str, default='can0', help='CAN interface to use')
    parser.add_argument('--target', type=str, metavar='THROTTLE', help='Target node to update', required=True)
    parser.add_argument('--bin', type=str, metavar='firmware.bin', help='Firmware binary to send', required=True)
    args = parser.parse_args()

    global chunk

    coloredlogs.install(level='info')
    log = logging.getLogger('main')

    cand_bus = Bus()
    isotp_tx_id = cand_bus.get_message_id(f'UPD_IsoTpTx_{args.target}')
    isotp_rx_id = cand_bus.get_message_id(f'{args.target}BL_IsoTpTx')
    print(f"Using isotp_tx_id {isotp_tx_id} and isotp_rx_id {isotp_rx_id}.")

    isotp_stack = isotp.CanStack(
        can.interface.Bus('can0', bustype = 'socketcan'),
        address = isotp.Address(isotp.AddressingMode.Normal_11bits, rxid=isotp_rx_id, txid=isotp_tx_id),
    )
    log.info('Created isotp stack')

    firmware = open(args.bin, 'rb').read()
    total_size = len(firmware)
    log.info(f'Read firmware binary ({total_size} bytes).')

    node = TargetNode(args.target, total_size)
    log.info(f'Tracking target node {node.node}')

    log.info(f'Waiting for {node.node}BL to come up...')
    init_start = datetime.datetime.now()
    while not node.is_alive():
        if datetime.datetime.now() - init_start > datetime.timedelta(seconds=10):
            log.error(f"Node {node.node}BL did not come up. Exiting.")
            exit(-1)

        sleep(SLEEP_WAIT)

    log.info(f"Waiting for node to be in RECV_CHUNK....")
    while node.state() != 'RECV_CHUNK':
        sleep(SLEEP_WAIT)

    sleep(SLEEP_WAIT)

    log.info('Begin sending firmware image.')
    for i, _ in enumerate(trange(0, len(firmware), CHUNK_SIZE)):
        chunk = i

        start = i * CHUNK_SIZE
        end = min(start + CHUNK_SIZE, total_size)

        log.debug(f"Sending chunk::: {start}...{end}")

        chunk_data = firmware[start:end]
        isotp_stack.send(chunk_data)


        log.debug('Waiting for isotp stack to be done transmitting')
        while isotp_stack.transmitting():
            isotp_stack.process()

        log.debug('Waiting for node to be in RECV_CHUNK again')
        while True:
            bl_state = node.state()

            if bl_state == 'RECV_CHUNK':
                sleep(0.1) # give the node some time, but this shouldn't be needed
                break
            elif bl_state == 'FAULT':
                log.error('Node is in FAULT; aborting.')
                node.stop()
                exit(-1)
            elif bl_state not in ['COMMIT_CHUNK', 'FINALIZE', 'CHECK_DESC']:
                log.error(f'Node in unexpected state {bl_state}; aborting.')
                exit(-1)
            else:
                sleep(SLEEP_WAIT)

    # stop the updatecontrol transmission right now so we don't trigger another update
    # once we're back in the firmware
    node.stop()

    log.info('Update done; waiting for node to exit bootloader.')
    while node.is_alive():
        if node.state() == 'FAULT':
            log.error('Node is in FAULT; update might have failed')
            exit(-1)

    log.info('Done!')
    exit(0)


if __name__ == "__main__":
    main()
