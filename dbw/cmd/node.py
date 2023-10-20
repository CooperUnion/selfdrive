from pprint import pprint

import cantools
import can


class DBWNode:
    name = "UNDEF"
    status = "UNDEF"  # should correspond to undefined

    def __init__(self, name: str, id: int, db, bus):
        self.name = name
        self.id = id
        self.db = db
        self.bus = bus
        self.status_msg = db.get_message_by_name(f"dbwNode_Status_{self.name}")
        self.syscmd_msg = db.get_message_by_name(f"dbwNode_SysCmd_{self.name}")

    def set_status(self, status):
        self.status = status

    def get_status(self):
        return self.status

    def send_msg(self, msg, data):
        cmd = msg.encode(data)
        msg = can.Message(
            arbitration_id=msg.frame_id, data=cmd, is_extended_id=False
        )
        self.bus.send(msg)

    def send_syscmd(self, data):
        cmd = self.syscmd_msg.encode(data)
        msg = can.Message(
            arbitration_id=self.syscmd_msg.frame_id,
            data=cmd,
            is_extended_id=False,
        )
        self.bus.send(msg)

    def send_estop(self):
        self.send_syscmd({'DbwActive': 0, 'ESTOP': 1})

    def send_active(self):
        self.send_syscmd({'DbwActive': 1, "ESTOP": 0})


class Accel(DBWNode):
    modectrl_status = 0

    def __init__(self, name: str, id: int, db, bus, custom_msgs):
        self.modectrl_msg = self.db.get_messsage_by_name(f"dbwNode_Accel_Cmd")
        self.disable_mode_control()
        super.__init__(name, id, db, bus, custom_msgs)

    def enable_mode_control(self):
        self.send_msg(self.modectrl_msg, {"ThrottleCmd": 0, "ModeCtrl": 1})
        self.modectrl_status = 1

    def disable_mode_control(self):
        self.send_msg(self.modectrl_msg, {"ThrottleCmd": 0, "ModeCtrl": 0})
        self.modectrl_status = 0

    def send_throttle_command(self, cmd):
        self.send_msg(
            self.modectrl_msg,
            {"ThrottleCmd": cmd, "ModeCtrl": self.modectrl_status},
        )
