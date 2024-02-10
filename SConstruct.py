# ruff: noqa: F821

import os

import env as uenv

EnsureSConsVersion(4, 5, 2)
EnsurePythonVersion(3, 11)


build = 'build'


env = Environment(
    ENV={
        **uenv.ENV,
        'PATH': os.environ['PATH'],
        'TERM': os.environ.get('TERM'),
    },
    tools=[
        'default',
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
