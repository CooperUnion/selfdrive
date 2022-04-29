import can
import cantools
import time

db = cantools.database.load_file("./odrive-cansimple.dbc")

def get_msg_by_name(name: str) -> cantools.database.can.message.Message:
    return db.get_message_by_name(name)

inputvel_msg = get_msg_by_name("Set_Input_Vel")

bus = can.Bus("can0", bustype="socketcan")
axisID = 0x30

vel = 1

while True:

    data = inputvel_msg.encode({'Input_Torque_FF': 0, 'Input_Vel': vel})
    msg = can.Message(arbitration_id=axisID << 5 | 0x0D, is_extended_id=False, data=data)
    bus.send(msg)
    time.sleep(1)
    vel *= -1