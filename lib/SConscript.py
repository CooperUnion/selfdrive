# ruff: noqa: F821

Import('envs')


env = envs['env']
esp32s3 = envs['esp32s3']


for env in envs.values():
    env['LIBRARIES'] = {}


esp32s3.AppendUnique(
    CPPDEFINES='NODE_BOARD_2V0B',
    CPPPATH='ccmn_defs/ccmn-pins',
)


esp32s3.AppendUnique(
    CPPPATH=[
        f'ember/ember-{dir}'
        for dir in [
            'bltools',
            'can',
            'commonh',
            'tasking',
        ]
    ],
)
ember = esp32s3.StaticLibrary(
    'ember',
    [
        f'ember/ember-{src}'
        for src in [
            'bltools/ember_bltools.c',
            'can/ember_can.c',
            'tasking/ember_tasking.c',
            'tasking/tasking.c',
            'tasking/watchdog.c',
        ]
    ],
)
esp32s3['LIBRARIES']['ember'] = ember


lib = [
    'ember',
]

Return('lib')
