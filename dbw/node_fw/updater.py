#!/usr/bin/env python3

import argparse

import cand


def update(mod_name: str, bin_path: str):
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

    bus = cand.client.Bus()


if __name__ == '__main__':
    main()
