# ruff: noqa: F821

Import('env')


node = 'GAS'

opencan = env.OpenCan(
    network=env['CAN']['NETWORK'],
    node=node,
)

source = [
    env.StaticObject(
        src,
        CPPDEFINES=[
            ('EMBER_NODE_IDENTITY', node),
            '$CPPDEFINES',
        ],
        CPPPATH=[
            env.Dir(opencan[0].dir.name),
            '$CPPPATH',
        ],
    )
    for src in [
        'gas.c',
        'pedal.c',
        *env['LIBRARIES']['firmware-base'],
    ]
]

source += opencan
source += env['LIBRARIES']['ember']
source += env['LIBRARIES']['node-entry']

gas = env.StaticLibrary(node.lower(), source)[0]
firmware, flash = env.EspIdf(gas, 'esp32s3')


Return('firmware', 'flash')
