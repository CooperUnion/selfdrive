#!/usr/bin/env python3
# node simulator

import logging
import multiprocessing as mp

from cand.client import Bus

from node import Node


def main():
    # hello there

    names = [
        'Blink',
        'Throttle',
        'Brake',
        'Encoder',
        'RearEncoder',
        'PbMon',
    ]

    nodes = []

    for name in names:
        node = Node(name)
        nodes.append(mp.Process(target=node._run))

    for node in nodes:
        node.start()


if __name__ == '__main__':
    main()
