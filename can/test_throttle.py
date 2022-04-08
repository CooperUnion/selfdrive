#!/usr/bin/env python3

import can
import cantools

from pprint import pprint
from time import sleep

bus = can.interface.Bus('can0', bustype='socketcan')

db = cantools.database.load_file('igvc_can.dbc')
pprint(db.messages)

command_message = db.get_message_by_name('dbwNode_Accel_Cmd')
command_message2 = db.get_message_by_name('dbwNode_Accel_Cntrls_Cmd')

cmd_dbw = db.get_message_by_name('dbwNode_SysCmd')
out = cmd_dbw.encode({'DbwActive': 1, 'ESTOP': 0})
out_msg = can.Message(arbitration_id=cmd_dbw.frame_id, data=out, is_extended_id=False)
bus.send(out_msg)

step_input_magnitude = .40 #this is the %throttle for %input test
thttle_cmd = 0
for i in range(0, 500):
    if(i <= 50 and i>=0):
      thttle_cmd = 0 # make sure car isnt moving
    elif(i > 50 and i <= 400):
      thttle_cmd = step_input_magnitude #set step input
    elif (i > 400):
      thttle_cmd = 0 # make sure car isnt moving

    cmd = command_message.encode({'ThrottleCmd': thttle_cmd,'ModeCtrl': 1} )#mode control1 forces node to control throttle
    cmd2 = command_message2.encode({'TargetVel': 0 ,'Kp': 0, 'Ki': 0,'Kd': 0, 'CharMode': 1})
    msg1 = can.Message(arbitration_id=command_message.frame_id, data=cmd, is_extended_id=False)
    msg2 = can.Message(arbitration_id=command_message2.frame_id, data=cmd2, is_extended_id=False)
    bus.send(msg1)
    bus.send(msg2)
    sleep(0.05)
