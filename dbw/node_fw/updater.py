#!/usr/bin/env python3

import argparse


def main():
    argparser = argparse.ArgumentParser(description='node_fw updater')

    argparser.add_argument(
        '-m',
        '--module',
        help='module to flash',
    )

    args = argparser.parse_args()


if __name__ == '__main__':
    main()
