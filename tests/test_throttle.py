#!/usr/bin/env python3

import argparse
import time

from cand.client import Bus

bus = Bus()


def run_test(percent, duration):
    bus.send("dbwNode_Encoder_Data", {"Encoder0": 0, "Encoder1": 1, "Time": 100})
    bus.send("dbwNode_SysCmd", {"DbwActive": 1, "ESTOP": 0})
    bus.send(
        "dbwNode_Accel_Cmd", {"ThrottleCmd": min(percent, 100) / 100, "ModeCtrl": 1}
    )

    time_start = time.time()
    i = 0
    while True:
        if (time.time() - time_start) > abs(duration):
            break
        print(f"{i} {bus.get('dbwNode_Encoder_Data')}")
        i += 1
        time.sleep(0.01)


def end_test():
    bus.send("dbwNode_Accel_Cmd", {"ThrottleCmd": 0, "ModeCtrl": 0})
    bus.send("dbwNode_SysCmd", {"DbwActive": 0, "ESTOP": 0})


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

    run_test(args.percent, args.time)
    end_test()


if __name__ == "__main__":
    try:
        main()
    except:
        end_test()
