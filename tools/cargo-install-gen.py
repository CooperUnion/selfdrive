#!/usr/bin/env python3

import argparse
import pathlib
import shlex
import tomllib


def generate_commands(crates: dict[str, dict[str, str]]) -> list[str]:
    cmds = []

    for crate, options in crates.items():
        cmd = ['cargo', 'install']

        flags = [
            ('git', True),
            ('branch', True),
            ('tag', True),
            ('rev', True),
            ('locked', False),
        ]

        for flag, arg in flags:
            if option := options.get(flag):
                cmd += [f'--{flag}']

                if arg:
                    cmd += [shlex.quote(option)]

        cmd += [shlex.quote(crate)]

        cmds += [' '.join(cmd)]

    return cmds


if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        prog='cargo-install-gen.py',
        description='generate cargo install commands',
    )

    parser.add_argument(
        '-c',
        '--config',
        required=True,
        type=pathlib.Path,
    )

    args = parser.parse_args()

    with open(args.config, 'rb') as f:
        config = tomllib.load(f)

    cmds = generate_commands(config)

    print('\n'.join(cmds))
