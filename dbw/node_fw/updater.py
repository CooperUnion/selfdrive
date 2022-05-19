#!/usr/bin/env python3

import argparse

import cand


class Updater:
    def __init__(self, bus=cand.client.Bus()):
        self.bus = bus

    def update(self, mod_name: str, bin_path: str):
        bin = None
        with open(bin_path, 'rb') as bin_fp:
            bin = bin_fp.read()


def main():
    argparser = argparse.ArgumentParser(description='node_fw updater')

    argparser.add_argument(
        '-m',
        '--module',
        help='module to flash',
    )

    args = argparser.parse_args()

    updater = Updater()


if __name__ == '__main__':
    main()
