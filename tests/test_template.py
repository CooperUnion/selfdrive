#!/usr/bin/env python3

import argparse
import time

from cand.client import Bus


def main():
    parser = argparse.ArgumentParser(description="Test template")

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

    bus = Bus()


if __name__ == "__main__":
    main()
