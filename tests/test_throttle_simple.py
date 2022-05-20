#!/usr/bin/env python3

import argparse
import time

from cand.client import Bus

bus = Bus()


def main():
    parser = argparse.ArgumentParser(description="test throttle")

    parser.add_argument(
        "-p",
        "--percent",
        help="value between 0 and 100",
        metavar="n",
        type=int,
        required=True,
    )
    parser.add_argument(
        "-t",
        "--time",
        help="duration in seconds",
        metavar="n.n",
        type=float,
        required=True,
    )

    args = parser.parse_args()

    bus.send("dbwNode_Encoder_Data", {"Encoder0": 0, "Encoder1": 1, "Time": 100})
    bus.send("dbwNode_SysCmd", {"DbwActive": 1, "ESTOP": 0})
    bus.send(
        "dbwNode_Accel_Cmd",
        {"ThrottleCmd": min(args.percent, 100) / 100, "ModeCtrl": 1},
    )

    time.sleep(abs(args.time))

    bus.send("dbwNode_Accel_Cmd", {"ThrottleCmd": 0, "ModeCtrl": 0})
    bus.send("dbwNode_SysCmd", {"DbwActive": 0, "ESTOP": 0})


if __name__ == "__main__":
    try:
        main()
    except:
        bus.send("dbwNode_Accel_Cmd", {"ThrottleCmd": 0, "ModeCtrl": 0})
        bus.send("dbwNode_SysCmd", {"DbwActive": 0, "ESTOP": 0})
