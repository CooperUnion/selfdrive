#!/usr/bin/env python3

import argparse
import json
import os
import shlex
import typing


_CARGO_HOME = '.cargo'
_IDF_BUILD = '.esp-idf'
_IDF_TOOLS_PATH = '.espressif'
_VIRTUAL_ENV = '.venv'


DIRENV_INSTALLED: typing.Final[dict[str, str]] = {
    f'DIRENV_INSTALLED_{key}': f'{value}/.direnv.installed'
    for key, value in {
        'CRATES': _CARGO_HOME,
        'IDF_TOOLS': _IDF_TOOLS_PATH,
        'PYTHON_REQUIREMENTS': _VIRTUAL_ENV,
        'SCONS_ESP_IDF_ENVIRONMENT': _IDF_BUILD,
    }.items()
}


PATHS: typing.Final[dict[str, str]] = {
    key: os.path.abspath(value)
    for key, value in {
        **DIRENV_INSTALLED,
        'CARGO_HOME': _CARGO_HOME,
        'IDF_BUILD': _IDF_BUILD,
        'IDF_BUILD_SCONS_ESP32S3': f'{_IDF_BUILD}/scons/esp32s3.json',
        'IDF_PATH': 'lib/esp-idf',
        'IDF_TOOLS_PATH': _IDF_TOOLS_PATH,
        'REPO_ROOT': '.',
        'VIRTUAL_ENV': _VIRTUAL_ENV,
    }.items()
}


ENV: typing.Final[dict[str, str]] = {
    **PATHS,
    'CMAKE_GENERATOR': 'Ninja',
    'IDF_PYTHON_CHECK_CONSTRAINTS': 'no',
    'IDF_TARGETS': 'esp32s3',
    'VIRTUAL_ENV_DISABLE_PROMPT': 'true',
}


def idf(env, target: str):
    env = env.Clone()

    with open(PATHS[f'IDF_BUILD_SCONS_{target.upper()}']) as f:
        idf = json.load(f)

    # we do this to supress warnings out of our control
    includes = [f'-isystem{include}' for include in idf['includes']]

    env.AppendUnique(
        CPPDEFINES=idf['defines'],
        CCFLAGS=includes
        + idf['options']
        + idf['optimization']
        + idf['debug']
        + idf['mcu']
        + idf['std'],
    )

    env.PrependENVPath('PATH', idf['path'])

    prefix = idf['prefix']

    env.Replace(
        **{
            key: f'{prefix}{val}'
            for key, val in {
                'AR': 'ar',
                'AS': 'as',
                'CC': 'gcc',
                'CXX': 'g++',
                'RANLIB': 'ranlib',
                'SHLINK': 'ld',
            }.items()
        }
    )

    return env


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
