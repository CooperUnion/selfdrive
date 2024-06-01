import cand
import time


def set_brake_gains(Kp, Ki, Kd):
    bus = cand.client.Bus()
    bus.send(
        'DBW_SetBRAKEGains',
        {'DBW_gainKp': Kp, 'DBW_gainKi': Ki, 'DBW_gainKd': Kd},
    )

    print(f"Sent Kp: {Kp}, Ki: {Ki}, Kd: {Kd} to brake")


def set_ctrl_gains(Kp, Ki, Kd):
    bus = cand.client.Bus()
    bus.send(
        'DBW_SetCTRLVelocityGains',
        {'DBW_gainKp': Kp, 'DBW_gainKi': Ki, 'DBW_gainKd': Kd},
    )

    print(f"Sent Kp: {Kp}, Ki: {Ki}, Kd: {Kd} to ctrl")


def run_test(brake_percent, input_time):
    bus = cand.client.Bus()

    timestep = input_time / 0.005

    if input_time == 0:
        bp = brake_percent
    else:
        increment = brake_percent / timestep
        bp = 0
        while bp < brake_percent:
            print(bp)
            bus.send(
                'DBW_RawVelocityCommand',
                {'DBW_brakePercent': bp, 'DBW_throttlePercent': 0},
            )
            time.sleep(0.005)
            bp = bp + increment

    while True:
        print(bp)
        if bp > 100:
            bp = 100
        bus.send(
            'DBW_RawVelocityCommand',
            {'DBW_brakePercent': bp, 'DBW_throttlePercent': 0},
        )
        time.sleep(0.005)
