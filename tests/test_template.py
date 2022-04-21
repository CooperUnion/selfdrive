#!/usr/bin/env python3

import argparse

import igvcutils


def main():
    parser = argparse.ArgumentParser(description='test template')

    parser.add_argument(
        '-c',
        '--can',
        help='CAN device',
        metavar='canx',
    )
    parser.add_argument(
        '-d',
        '--dbc',
        help='CAN DBC',
        metavar='file.dbc',
    )

    args = parser.parse_args()

    bus = igvcutils.can.Bus(args.dbc, args.can)


if __name__ == '__main__':
    main()
