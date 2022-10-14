import time

import unit


units = [ ]


# message timeouts
DBWNODE_ACCEL_DATA_TIMEOUT_NS   = 200_000_000
DBWNODE_VEL_CMD_TIMEOUT_NS      = 200_000_000
DBWNODE_ENCODER_DATA_TIMEOUT_NS = 200_000_000

# unit timeouts
THROTTLE_DIFF_TIMEOUT_NS = 4_000_000_000

# unit limits
THROTTLE_PERCENT_DIFF_MAX    = 0.04
ENCODER_5MPH_TICKS           = 47
ENCODER_DELTA_TIMEOUT_MIN_US = 9_000
ENCODER_DELTA_TIMEOUT_MAX_US = 11_000


class UnitThrottle(unit.Unit):
    def __init__(self):
        super().__init__()

        self._throttle_diff_time_ns = None

    def test(self):
        data_rec = self._bus.get('DBW_NodeAccelData')
        if data_rec is None: return self.abort('TIMEOUT')

        cmd_rec = self._bus.get('DBW_NodeVelCmd')
        if cmd_rec is None: return self.abort('TIMEOUT')

        cur_time = time.time_ns()

        data_time, data_data = data_rec
        cmd_time, cmd_data   = cmd_rec

        if cur_time - data_time >= DBW_NODEACCELDATA_TIMEOUT_NS:
            return self.abort('TIMEOUT')
        if cur_time - cmd_time >= DBW_NODEVELCMD_TIMEOUT_NS:
            return self.abort('TIMEOUT')

        perc_diff = abs(data_data['percent'] - cmd_data['throttlePercent'])

        if perc_diff > THROTTLE_PERCENT_DIFF_MAX:
            if self._throttle_diff_time_ns is None:
                self._throttle_diff_time_ns = cur_time
                return True

            if cur_time - self._throttle_diff_time_ns >= THROTTLE_DIFF_TIMEOUT_NS:
                self._throttle_diff_time_ns = None
                return self.abort('INVALID_STATE')

        return True

units.append(UnitThrottle())


class UnitVelocity(unit.Unit):
    def test(self):
        encoder_rec = self._bus.get('ENCF_EncoderData')
        if encoder_rec is None: return self.abort('TIMEOUT')

        cur_time = time.time_ns()

        encoder_time, encoder_data = encoder_rec

        if cur_time - encoder_time >= ENCF_ENCODERDATA_TIMEOUT_NS:
            return self.abort('TIMEOUT')

        encoder_max = max(
            abs(encoder_data['encoderLeft']),
            abs(encoder_data['encoderRight']),
        )

        if encoder_data['dtUs'] <= ENCODER_DELTA_TIMEOUT_MIN_US:
            return self.abort('TIMEOUT')

        if encoder_data['dtUs'] >= ENCODER_DELTA_TIMEOUT_MAX_US:
            return self.abort('TIMEOUT')

        if encoder_max > ENCODER_5MPH_TICKS:
            return self.abort('LIMIT_EXCEEDED')

        return True

units.append(UnitVelocity())
