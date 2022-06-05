#!/usr/bin/env python3

import argparse
import logging
import time

import cand
import coloredlogs

import node


NODE_STATUS_CYCLE_TIME_S = 0.01


def main():
    argparser = argparse.ArgumentParser(description='node daemon')

    argparser.add_argument(
        '--can-dbwEnable',
        action='store_true',
        default=False,
        help='enable \'dbwEnable\' message',
    )
    argparser.add_argument(
        '--redis-host',
        default='localhost',
        help='hostname of redis server',
        metavar='localhost',
    )
    argparser.add_argument(
        '--redis-port',
        default='6379',
        help='port of redis server',
        metavar='6379',
    )

    args = argparser.parse_args()

    bus = cand.client.Bus(
        redis_host=args.redis_host,
        redis_port=args.redis_port,
    )

    coloredlogs.install()
    logger = logging.getLogger('nodesd')

    mod_ident = [
        'Blink',
        'Throttle',
        'Brake',
        'Encoder',
        'RearEncoder',
        'PbMon',
        'Steering'
    ]

    nodes = [node.Node(mod, bus=bus) for mod in mod_ident]

    logger.info('initialized node logger(s)')

    dbw_enable = node.DbwEnable(
        bus=cand.client.Bus(
            redis_host = args.redis_host,
            redis_port = args.redis_port,
        ),
    )

    if args.can_dbwEnable:
        dbw_enable.start()
        logger.info('listening for \'dbwEnable\' message')

    logger.info('starting servicer for node(s)')
    while True:
        for mod in nodes:
            mod.service()

        time.sleep(NODE_STATUS_CYCLE_TIME_S)

if __name__ == '__main__':
    main()
