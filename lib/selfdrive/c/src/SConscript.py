# ruff: noqa: F821

Import('env')


sources = [
    env.File(file)
    for file in [
        'pid.c',
    ]
]


Return('sources')
