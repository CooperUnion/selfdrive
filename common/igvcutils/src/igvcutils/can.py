import csv

import can
import cantools

from functools import cache


class DBC:
    def __init__(self, dbc_path: str):
        self._dbc = cantools.database.load_file(dbc_path)

    def dump2csv(self, dump_path: str, out_path: str):
        fields = ['unix time (s)', 'delta (s)']

        candump = None
        with open(dump_path) as fp:
            candump = fp.readlines()

        init_time = float(candump[0].split()[0].strip('()'))

        decoded = []
        for raw in candump:
            msg = {}
            raw = raw.split()

            unix_time = float(raw[0].strip('()'))

            msg['unix time (s)'] = unix_time
            msg['delta (s)'] = unix_time - init_time

            frame_id = int(raw[2], 16)
            raw_data = bytearray.fromhex(''.join(raw[4:]))

            try:
                name = self.get_msg_by_frame_id(frame_id).name
            except KeyError:
                continue
            data = self._dbc.decode_message(frame_id, raw_data)

            for key, val in data.items():
                key = name + '_' + key

                msg[key] = val

                if key not in fields:
                    fields.append(key)

            decoded.append(msg)

        with open(out_path, 'w+') as fp:
            writer = csv.DictWriter(fp, fieldnames=fields)
            writer.writeheader()
            writer.writerows(decoded)

    @cache
    def get_msg_by_frame_id(
        self, frame_id: int
    ) -> cantools.database.can.message.Message:
        return self._dbc.get_message_by_frame_id(frame_id)

    @cache
    def get_msg_by_name(
        self, name: str
    ) -> cantools.database.can.message.Message:
        return self._dbc.get_message_by_name(name)


class Bus:
    def __init__(self, dbc: DBC, can_iface: str, bustype: str = 'socketcan'):
        self._bus = can.interface.Bus(can_iface, bustype=bustype)
        self._dbc = dbc

    def send(self, message: str, signals: dict):
        msg = self._dbc.get_msg_by_name(message)
        data = msg.encode(signals)
        out = can.Message(
            arbitration_id=msg.frame_id, data=data, is_extended_id=False
        )

        self._bus.send(out)


def endianswap(
    val: int, byteorder: str, *, src_signed=False, dst_signed=False
) -> int:
    swap_byteorder = None
    if byteorder == 'little':
        swap_byteorder = 'big'
    elif byteorder == 'big':
        swap_byteorder = 'little'
    else:
        raise ValueError("byteorder must be either 'little' or 'big'")

    length = val.bit_length() // 8
    if val.bit_length() % 8:
        length += 1

    raw = val.to_bytes(length, byteorder, signed=src_signed)
    return int.from_bytes(raw, swap_byteorder, signed=dst_signed)
