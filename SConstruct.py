# ruff: noqa: F821

import os

import env as uenv

EnsureSConsVersion(4, 5, 2)
EnsurePythonVersion(3, 11)


build = 'build'


AddOption(
    '--esp-baud',
    default='',
    help='serial port baud rate for flashing/reading',
    metavar='BAUD',
    type=str,
)
AddOption(
    '--esp-port',
    default='',
    help='serial port device',
    metavar='PORT',
    type=str,
)


env = Environment(
    ENV={
        **uenv.ENV,
        'PATH': os.environ['PATH'],
        'TERM': os.environ.get('TERM'),
    },
    ESPBAUD=GetOption('esp_baud'),
    ESPPORT=GetOption('esp_port'),
    tools=[
        'default',
        'Component',
        'EspIdf',
        'Phony',
    ],
)
env.AppendUnique(
    CCFLAGS=[
        '-Wall',
        '-Wextra',
        '-Wpedantic',
        '-g',
        '-std=gnu17',
    ]
)

esp32s3 = uenv.idf(env, 'esp32s3')

envs = {
    'env': env,
    'esp32s3': esp32s3,
}

Export('env')
Export('envs')


can = env.SConscript(
    'can/SConscript.py',
    variant_dir=f'{build}/can',
    duplicate=False,
)
components = env.SConscript(
    'components/SConscript.py',
    variant_dir=f'{build}/components',
    duplicate=False,
)
ember_bl = env.SConscript(
    'dbw/ember_bl/SConscript.py',
    variant_dir=f'{build}/dbw/ember_bl',
    duplicate=False,
)
node_fw = env.SConscript(
    'dbw/node_fw/SConscript.py',
    variant_dir=f'{build}/dbw/node_fw',
    duplicate=False,
)
