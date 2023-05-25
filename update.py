
import can
import coloredlogs
import isotp
import logging

from cand.client import Bus
from threading import Thread
from time import sleep
from tqdm import trange

CHUNK_SIZE = 4095
iface = 'can0'

SLEEP_WAIT = 0.01

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

            self.bus.send('UPD_UpdateControl', signals)
            sleep(0.02)

    def state(self) -> str:
        message = f'{self.node}_Status'
        data = self.bus.get_data(message)
        if data is None:
            self.log.error(f"Missing {message} from cand... is the node present?")
            exit(-1)

        return data[f'{self.node}_state']

    def is_alive(self) -> bool:
        message = f'{self.node}_Status'
        dt = self.bus.get_time_delta(message)

        if dt is None:
            return False

        ALIVE_DELTA_NS = 500 * 1000 * 1000 # 0.5 seconds
        return dt <= ALIVE_DELTA_NS


def main():
    global chunk

    coloredlogs.install(level='info')
    log = logging.getLogger('main')

    isotp_stack = isotp.CanStack(
        can.interface.Bus(iface, bustype = 'socketcan'),
        address = isotp.Address(isotp.AddressingMode.Normal_29bits, rxid=0x301, txid=0x330),
    )
    log.info('Created isotp stack')

    firmware = open('dbw/node_fw/.pio/build/blink2.0A/firmware.bin', 'rb').read()
    total_size = len(firmware)
    log.info(f'Read firmware binary ({total_size} bytes).')

    node = TargetNode('TESTBL', total_size)
    log.info(f'Tracking target node {node.node}')

    log.info(f'Waiting for {node.node} to come up...')
    while not node.is_alive():
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
                sleep(0.005) # give the node some time, but this shouldn't be needed
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
