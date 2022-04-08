#!/usr/bin/env python3
import can
import cantools

from pprint import pprint
from time import sleep

bus = can.interface.Bus('can0', bustype='socketcan')

db = cantools.database.load_file('igvc_can.dbc')
pprint(db.messages)


command_message1 = db.get_message_by_name('dbwNode_Accel_Cmd')
command_message = db.get_message_by_name('dbwNode_Accel_Cntrls_Cmd')

cmd_dbw = db.get_message_by_name('dbwNode_SysCmd')
out = cmd_dbw.encode({'DbwActive': 1, 'ESTOP': 0})
out_msg = can.Message(arbitration_id=cmd_dbw.frame_id, data=out, is_extended_id=False)
bus.send(out_msg)

target_velocity = 0
target_velocity_magnitude = 0.225 #will change but for testing will stay constant
x = 50 #for now

for i in range(0, 500):
    if(i <= 50 and i>0):
      target_velocity = 0 # make sure car isnt moving
    elif(i > 50 and i <= 400):
      target_velocity = target_velocity_magnitude #set step input
    elif (i > 400):
      target_velocity = 0 # make sure car isnt moving

    cmd1 = command_message1.encode({'ThrottleCmd': 0, 'ModeCtrl': 1} )#mode control1 forces node to control throttle
    cmd = command_message.encode({'TargetVel': target_velocity ,'Kp': x, 'Ki': 0,'Kd': 0, 'CharMode': 0})
    msg1 = can.Message(arbitration_id=command_message1.frame_id, data=cmd1, is_extended_id=False)
    msg = can.Message(arbitration_id=command_message.frame_id, data=cmd, is_extended_id=False)
    bus.send(msg1)
    bus.send(msg)
    sleep(0.05)
