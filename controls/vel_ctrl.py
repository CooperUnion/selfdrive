
import igvcutils
from cand.client import Client
from PID_beard import PIDController

class vel_ctrl:

    def __init__(self):
        self.controller = PIDController(kp=2, ki=.1, kd=.1, )
        self.bus = Client()

    def ctrl_from_twist(self, twist_message):
        v_des = twist.linear.x
        enc = self.bus.get(cand info idk)
        time = self.bus.get(cand info time)
        v_actual = self.enc_to_velocity(enc, time)
        ctrl_out = PIDController(v_des, v_actual)
        pedal_percentage = self.acc_to_pedal(ctrl_out)
        self.bus.send('dbwNode_Accel_Cmd', {'ThrottleCmd': pedal_pecentage, 'ModeCtrl': 1})

    def ctrl_vel_fixed(self):
        v_des = 2.2352
        enc = self.bus.get(cand info idk)
        # time = self.bus.get(cand info time)
        v_actual = self.enc_to_velocity(enc, 0.01)
        ctrl_out = PIDController(v_des, v_actual)
        pedal_percentage = self.acc_to_pedal(ctrl_out)
        self.bus.send('dbwNode_Accel_Cmd', {'ThrottleCmd': pedal_pecentage, 'ModeCtrl': 1})

    def acc_to_pedal(self, acceleration):
        return 1*acceleration

    def enc_to_velocity(self, enc, time):
        enc_ticks = 4000
        wheel_circumference = 1.899156
        meters_per_tick = wheel_circumference/enc_ticks
        return (meters_per_ticks*enc)/time

if __name__ == "__main__":
    c = vel_ctrl()
    while(1):
        c.ctrl_vel_fixed()
