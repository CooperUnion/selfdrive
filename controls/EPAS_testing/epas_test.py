import can
import cantools
import time

db = cantools.database.load_file("./odrive-cansimple.dbc")

def get_msg_by_name(name: str) -> cantools.database.can.message.Message:
    return db.get_message_by_name(name)

def get_controls(reference, actual):
    kp = 0.1 #Proportional gain of the position loop, 
    return (reference-actual) * kp # return the controller output (velocity command to odrive in turns/sec)

#convert the raw_count data from the absolute encoder to a useful unit (either in "turns", "degrees", or "radians"
def counts_to_turns(raw_count):
    ppr = 5760 # pulse per rev of the absolute encoder
    pass

bus = can.Bus("can0", bustype="socketcan")
axisID = 0x30

#ID 0x01
heartbeat_msg = get_msg_by_name("Heartbeat")
#ID 0x09
odrive_encoderdata_msg = get_msg_by_name("Get_Encoder_Estimates")
#ID 0x0B
contollermode_msg = get_msg_by_name("Set_Controller_Mode")
#ID 0x0C
inputpos_msg = get_msg_by_name("Set_Input_Pos")
#ID 0x0D
inputvel_msg = get_msg_by_name("Set_Input_Vel")
#ID 0x0F
limits_msg = get_msg_by_name("Set_Limits")

# print("Clearing Errors on axis: " + str(axisID))
# msg = can.Message(arbitration_id=axisID << 5 | 0x18)
# print(msg)

print("Requesting AXIS_STATE_FULL_CALIBRATION_SEQUENCE (0x03) on axisID: " + str(axisID))
msg = can.Message(arbitration_id=axisID << 5 | 0x07, data=[3, 0, 0, 0, 0, 0, 0, 0], dlc=8, is_extended_id=False)
print(msg)

try:
    bus.send(msg)
    print("Message sent on {}".format(bus.channel_info))
except can.CanError:
    print("Message NOT sent!  Please verify can0 is working first")

print("Waiting for calibration to finish...")
# Read messages infinitely and wait for the right ID to show up
while True:
    msg = bus.recv()
    if msg.arbitration_id == (axisID << 5 | 0x01):
        data = heartbeat_msg.decode(msg.data)
        print(data)
        if data['Axis_State'] == 0x1:
            print("\nAxis has returned to Idle state.")
            break

for msg in bus: 
    if(msg.arbitration_id == (axisID << 5 | 0x01)):
        errorCode = msg.data[0] | msg.data[1] << 8 | msg.data[2] << 16 | msg.data[3] << 24
        print("\nReceived Axis heartbeat message:")
        if errorCode == 0x0:
            print("No errors")
        else:
            print("Axis error!  Error code: "+str(hex(errorCode)))
        break

print("\nPutting axis",axisID,"into AXIS_STATE_CLOSED_LOOP_CONTROL (0x08)...")
msg = can.Message(arbitration_id=axisID << 5 | 0x07, data=[8, 0, 0, 0, 0, 0, 0, 0], dlc=8, is_extended_id=False)
print(msg)

# print("\nPutting axis",axisID,"into INPUT_MODE_PASSTHROUGH ")
# data = contollermode_msg.encode({'Input_Mode':0x01, 'Control_Mode':0x02})
# msg = can.Message(arbitration_id=axisID << 5 | 0x0B, data=data)
# print(msg)

try:
    bus.send(msg)
    print("Message sent on {}".format(bus.channel_info))
except can.CanError:
    print("Message NOT sent!")

for msg in bus:
    if msg.arbitration_id == (axisID << 5 | 0x01):
        print("\nReceived Axis heartbeat message:")
        if msg.data[4] == 0x8:
            print("Axis has entered closed loop")
        else:
            print("Axis failed to enter closed loop")
        break

data = limits_msg.encode({'Velocity_Limit':10.0, 'Current_Limit':10.0})
msg = can.Message(arbitration_id=axisID << 5 | 0x0F, is_extended_id=False, data=data)
bus.send(msg)

#Initialize init and previous position to None
prev_pos = None
prev_vel = None
init_pos = None
init_vel = None
desired_pos = None
delta_desired_pos = None
frequency = 0.25 
t0 = None

for msg in bus:
    if msg.arbitration_id == (axisID << 5 | 0x09):
        data = odrive_encoderdata_msg.decode(msg.data)
        prev_pos = data['Pos_Estimate']
        prev_vel = data['Vel_Estimate']
        #print(f"Position: {prev_pos}")
        #print(f"Velocity: {prev_vel}")
    if msg.arbitration_id == 0x1E5:
        pos_masked = int.from_bytes(msg.data[1:3], "big", signed = True) 
        #print(f"Absolute Encoder pos: {pos_masked}")
    if msg.arbitration_id == (axisID << 5 | 0x0D):
        data = inputvel_msg.decode(msg.data)
        vel = data['Input_Vel']
        print(f"Input Velocity: {vel}")

