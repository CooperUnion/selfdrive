#!/usr/bin/env python3

import argparse


def main():
    argparser = argparse.ArgumentParser(description='send_launch')

    argparser.add_argument(
        "--kp",
        default=0.0,
        metavar="n",
        type=float,
    )
    argparser.add_argument(
        "--ki",
        default=0.0,
        metavar="n",
        type=float,
    )
    argparser.add_argument(
        "--kd",
        default=0.0,
        metavar="n",
        type=float,
    )
    argparser.add_argument(
        "--vd",
        default=0.0,
        metavar="n",
        type=float,
    )

    args = argparser.parse_args()


if __name__ == '__main__':
    main()
