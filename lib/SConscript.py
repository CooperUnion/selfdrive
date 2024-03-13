# ruff: noqa: F821

Import('envs')


for env in envs.values():
    env['LIBRARIES'] = {}


env = envs['env']
esp32s3 = envs['esp32s3']


esp32s3.AppendUnique(
    CPPDEFINES='NODE_BOARD_2V0B',
    CPPPATH=esp32s3.Dir('ccmn_defs/ccmn-pins'),
)


esp32s3.AppendUnique(
    CPPPATH=[
        esp32s3.Dir(f'ember/ember-{dir}')
        for dir in [
            'bltools',
            'can',
            'commonh',
            'tasking',
        ]
    ],
)
ember = [
    esp32s3.File(f'ember/ember-{src}')
    for src in [
        'bltools/ember_bltools.c',
        'can/ember_can.c',
        'tasking/ember_tasking.c',
        'tasking/tasking.c',
        'tasking/watchdog.c',
    ]
]
esp32s3['LIBRARIES']['ember'] = ember


firmware_base = env.SConscript(
    'firmware-base/SConscript.py',
    exports={'env': esp32s3},
)
esp32s3['LIBRARIES']['firmware-base'] = firmware_base


node_entry = env.SConscript(
    'node-entry/SConscript.py',
    exports={'env': esp32s3},
)
esp32s3['LIBRARIES']['node-entry'] = node_entry


lib = [
    ember,
    firmware_base,
    node_entry,
]

Return('lib')
