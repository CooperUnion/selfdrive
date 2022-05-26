import pid

import cand


class Ctrl:
    ENCODER_TICKS_PER_ROTATION = 4000
    WHEEL_CIRCUMFRANCE_M       = 1.899156

    ENCODER_CAN_MESSAGE_NAME = 'dbwNode_Encoder_Data'
    ENOCDER0_SIGNAL_NAME     = 'Encoder0'
    ENOCDER1_SIGNAL_NAME     = 'Encoder1'
    TIME_DETLA_US_NAME       = 'Time'

    def __init__(self, pid: pid.Controller, bus: cand.client.Bus):
        self._pid = pid
        self._bus = bus
