#!/usr/bin/env python3

import logging
import time

import coloredlogs

from node import Node


def main():
    coloredlogs.install(level=logging.DEBUG)

    names = [
        'Blink',
        'Throttle',
        'Brake',
        'Encoder',
        'RearEncoder',
    ]

    nodes = []

    for name in names:
        nodes.append(Node(name))

    while True:
        for node in nodes:
            node.service()
        time.sleep(0.05)


if __name__ == '__main__':
    main()
