
import igvcutils
from cand.client import Client
from PID_beard import PIDController

class vel_ctrl:

    def __init__(self):
        self.controller = PIDController(kp=1, ki=.1, kd=.1, )
        self.bus = Client()

    def ctrl_from_twist(self, twist_message):
        v_des = twist.linear
        enc = self.bus.get(cand info idk)
        time = self.bus.get(cand for time)
        v_actual = self.enc_to_velocity(enc, time)
        ctrl_out = PIDController(v_des, v_actual)
        pedal_percentage = self.acc_to_pedal(ctrl_out)
        self.bus.send('dbwNode_Accel_Cmd', {'ThrottleCmd': pedal_pecentage, 'ModeCtrl': 1})

    def acc_to_pedal(self, acceleration):
        return 1*acceleration

    def enc_to_velocity(self, enc, time):
        enc_ticks = 4000
        wheel_circumference = 1.899156
        meters_per_tick = wheel_circumference/enc_ticks
        return (meters_per_ticks*enc)/time*100000
