# ruff: noqa: F821

Import('env')

node = 'BATT'

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
        'batt.c',  # Ensure this file exists in your source directory
        *env['LIBRARIES']['firmware-base'],
    ]
]

# Correct the typo in the variable name
source += opencan
source += env['LIBRARIES']['ember']
source += env['LIBRARIES']['node-entry']
source += env['LIBRARIES']['selfdrive']

batt = env.StaticLibrary(node.lower(), source)[0]
firmware, flash = env.EspIdf(batt, 'esp32s3')

Return('firmware', 'flash')
