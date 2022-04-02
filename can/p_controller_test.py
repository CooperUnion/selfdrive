#!/usr/bin/env python3
import can
import cantools

from pprint import pprint
from time import sleep

bus = can.interface.Bus('can0', bustype='socketcan')

db = cantools.database.load_file('/home/isabellaflynn/selfdrive/can/igvc_can.dbc')
pprint(db.messages)


command_message = db.get_message_by_name('dbwNode_Accel_Cntrls_Cmd')

target_velocity = 0
target_velocity_magnitude = 0.447 #will change but for testing will stay constant
x = 0 #for now

for i in range(0, 100):
    if(i <= 50 and i>0):
      target_velocity = 0 # make sure car isnt moving
    elif(i > 50 and i <= 90):
      target_velocity = target_velocity_magnitude #set step input
    elif (i > 90):
      target_velocity = 0 # make sure car isnt moving

    cmd = command_message.encode({'TargetVel': target_velocity ,'Kp': x, 'Ki': 0,'Kd': 0, 'CharMode': 1})
    msg = can.Message(arbitration_id=command_message.frame_id, data=cmd, is_extended_id=False)
    bus.send(msg)
    sleep(0.05)
