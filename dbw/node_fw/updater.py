#!/usr/bin/env python3

import argparse

import cand


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
