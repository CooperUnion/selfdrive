# ruff: noqa: F821

Import('env')


node = 'THROTTLE'

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
        'pedal.c',
        'throttle.c',
        *env['LIBRARIES']['firmware-base'],
    ]
]

source += opencan
source += env['LIBRARIES']['ember']
source += env['LIBRARIES']['node-entry']

throttle = env.StaticLibrary(node.lower(), source)[0]
firmware, flash = env.EspIdf(throttle, 'esp32s3')


Return('firmware', 'flash')
