import can
import cantools

from functools import cache


class Bus:
    def __init__(self, dbc_path: str, can_iface: str, bustype: str='socketcan'):
        self._bus = can.interface.Bus(can_iface, bustype=bustype)
        self._dbc = cantools.database.load_file(dbc_path)

    @cache
    def _get_msg_by_name(self, name: str) -> cantools.database.can.message.Message:
        return self._dbc.get_message_by_name(name)

    def send(self, message: str, signals: dict):
        msg  = self._get_msg_by_name(message)
        data = msg.encode(signals)
        out  = can.Message(arbitration_id=msg.frame_id, data=data, is_extended_id=False)

        self._bus.send(out)
