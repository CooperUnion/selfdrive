import numpy as np
import igvcutils
from cand.client import Client

bus = Client()
class vel_ctrl:
  def __init__():
    self.controller = PIDControl(kp, ki, kd)
  def ctrl_from_twist(twist_message):
    v_des = twist.linear
    v_actual = bus.get(cand info idk)*conversions which we have already.. luckily
    ctrl_out = PIDController(v_des, v_actual)
    bus.send('dbwNode_Accel_Cmd', {'ThrottleCmd': ctrl_out ,'Kp':1, 'Ki': 0,'Kd': 0, 'CharMode': 1})
