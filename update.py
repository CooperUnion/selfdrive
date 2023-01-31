
import can
import coloredlogs
import isotp
import logging

from cand.client import Bus
from threading import Thread
from time import sleep
from tqdm import trange

CHUNK_SIZE = 4095
node = 'TESTBL'
iface = 'can0'

chunk = 0

class TargetNode():
    def __init__(self, node: str, total_size: int):
        self.bus = Bus()
        self.node = node
        self.total_size = total_size
        self.update_control_thread = Thread(target = self.update_control)
        self.log = logging.getLogger(f"target_node_{node}")

        self.update_control_thread.start()
        self.log.info("Started UpdateControl thread.")

    def update_control(self):
        while True:
            signals = {
                'UPD_updateSizeBytes': self.total_size,
                'UPD_currentIsoTpChunk': chunk
            }

            self.bus.send('UPD_UpdateControl', signals)
            sleep(0.02)

    def state(self):
        message = f'{self.node}_Status'
        data = self.bus.get_data(message)
        if data is None:
            self.log.error(f"Missing {message} from cand... is the node present?")
            exit(-1)

        return data[f'{self.node}_state']

def main():
    global chunk

    coloredlogs.install(level='info')
    log = logging.getLogger('main')

    isotp_stack = isotp.CanStack(
        can.interface.Bus(iface, bustype = 'socketcan'),
        address = isotp.Address(isotp.AddressingMode.Normal_29bits, rxid=0x301, txid=0x330)
    )
    log.info('Created isotp stack')

    firmware = open('dbw/node_fw/.pio/build/blink2/firmware.bin', 'rb').read()
    total_size = len(firmware)
    log.info(f'Read firmware binary ({total_size} bytes).')

    node = TargetNode('TESTBL', total_size)
    log.info(f'Tracking target node TESTBL.')

    log.info(f'Waiting for node to be in RECV_CHUNK...')
    while node.state() != 'RECV_CHUNK':
        print(f"Waiting for node to be in RECV.... current is {node.state()}")
        sleep(0.2)

    log.info('Begin sending firmware image.')
    for i, _ in enumerate(trange(0, len(firmware), 4095)):
        chunk = i

        start = i * CHUNK_SIZE
        end = min(start + CHUNK_SIZE, total_size)

        log.info(f"Sending chunk::: {start}...{end}")

        chunk_data = firmware[start:end]
        isotp_stack.send(chunk_data)

        log.debug('Waiting for isotp stack to be done transmitting')
        while isotp_stack.transmitting():
            isotp_stack.process()
            sleep(isotp_stack.sleep_time())

        log.debug('Waiting for node to be in RECV_CHUNK again')
        while True:
            bl_state = node.state()

            if bl_state == 'RECV_CHUNK':
                break
            elif bl_state != 'COMMIT_CHUNK':
                log.error('Node not in expected state!')
                exit(-1)
            else:
                sleep(0.05)

    exit(0)


if __name__ == "__main__":
    main()
