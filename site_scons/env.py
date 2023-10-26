#!/usr/bin/env python3

import argparse
import os
import shlex
import typing


PATHS: typing.Final[dict[str, str]] = {
    key: os.path.abspath(value)
    for key, value in {
        'CARGO_HOME': '.cargo',
        'REPO_ROOT': '.',
        'VIRTUAL_ENV': '.venv',
    }.items()
}


ENV: typing.Final[dict[str, str]] = {
    **PATHS,
}


if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        prog='env.py',
        description='generate environment variables',
    )

    parser.add_argument(
        '-e',
        '--export',
        action='store_true',
    )

    args = parser.parse_args()

    export = 'export ' if args.export else ''

    for key, value in ENV.items():
        print(f'{export}{key}={shlex.quote(value)}')
