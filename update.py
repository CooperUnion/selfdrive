import can
import cantools
from time import sleep

bus = can.interface.Bus('can0', bustype = 'socketcan')
db = cantools.database.load_file('build/can/igvc_can.dbc')

node = 'TESTBL'
update_control = db.get_message_by_name('UPD_UpdateControl')

while True:
    data = update_control.encode({'UPD_updateSizeBytes': 10000,
                                     'UPD_currentIsoTpChunk': 0})
    can_message = can.Message(arbitration_id=update_control.frame_id, data=data)
    bus.send(can_message)
    sleep(1.0 / update_control.cycle_time)
