#!/usr/bin/env python3

import can
import cantools

from pprint import pprint
from time import sleep

bus = can.interface.Bus('vcan0', bustype='socketcan')

db = cantools.database.load_file('/home/isabellaflynn/selfdrive/can/igvc_can.dbc')
pprint(db.messages)

command_message = db.get_message_by_name('dbwNode_Accel_Cmd')

for i in range(0, 60):
    i = float(i) / 100.0
    cmd = command_message.encode({'ThrottleCmd': i, 'ModeCtrl': 1})
    msg = can.Message(arbitration_id=command_message.frame_id, data=cmd, is_extended_id=False)
    bus.send(msg)
    sleep(0.1)

command_message2 = db.get_message_by_name('dbwNode_Accel_Cntrls_Cmd')

step_input_magnitude = .05 #this is the %throttle for %input test
thttle_cmd = 0
for i in range(0, 100):
    if(i <= 50 and i>=0):
      thttle_cmd = 0 # make sure car isnt moving
    elif(i > 50 and i <= 90):
      thttle_cmd = step_input_magnitude #set step input
    elif (i > 90):
      thttle_cmd = 0 # make sure car isnt moving

    cmd1 = command_message.encode({'ThrottleCmd': thttle_cmd,'ModeCtrl': 1} )#mode control1 forces node to control throttle
    cmd2 = command_message2.encode({'TargetVel': 0 ,'Kp': 0, 'Ki': 0,'Kd': 0, 'CharMode': 0})
    msg1 = can.Message(arbitration_id=command_message.frame_id, data=cmd1, is_extended_id=False)
    msg2 = can.Message(arbitration_id=command_message2.frame_id, data=cmd2, is_extended_id=False)
    bus.send(msg1)
    bus.send(msg2)
    sleep(0.05)
