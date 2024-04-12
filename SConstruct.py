# ruff: noqa: F821

import os

import env as uenv

EnsureSConsVersion(4, 6, 0)
EnsurePythonVersion(3, 12)


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
AddOption(
    '--verbose',
    action='store_true',
    default=False,
    help='enable verbose output',
)


env = Environment(
    ENV={
        **uenv.ENV,
        'PATH': os.environ['PATH'],
        'TERM': os.environ.get('TERM'),
    },
    ESPBAUD=GetOption('esp_baud'),
    ESPPORT=GetOption('esp_port'),
    VERBOSE=GetOption('verbose'),
    tools=[
        'default',
        'Component',
        'EspIdf',
        'OpenCan',
        'Phony',
        'Project',
    ],
)
env.AppendUnique(
    CCFLAGS=[
        '-Wall',
        '-Wextra',
        '-Wpedantic',
        '-ggdb',
        '-std=gnu17',
    ]
)
env.AppendUnique(
    PDFLATEXFLAGS=[
        '--halt-on-error',
        '--shell-escape',
    ]
)
env.Replace(PDFLATEX='lualatex')

if not env['VERBOSE']:
    commands = [
        'AR',
        'CC',
        'RANLIB',
    ]
    for command in commands:
        env[f'{command}COMSTR'] = f'{command} $TARGET'

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
)
lib = env.SConscript(
    'lib/SConscript.py',
    variant_dir=f'{build}/lib',
)
components = env.SConscript(
    'components/SConscript.py',
    variant_dir=f'{build}/components',
)
ember_bl = env.SConscript(
    'dbw/ember_bl/SConscript.py',
    variant_dir=f'{build}/dbw/ember_bl',
    duplicate=False,
)
