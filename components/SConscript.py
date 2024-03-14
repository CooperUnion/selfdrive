# ruff: noqa: F821

Import('env')


components = env.SConscript(
    [
        f'{component}/SConscript.py'
        for component in [
            'bbc',
            'ctrl',
            'jk',
            'steer',
            'sup',
            'throttle',
        ]
    ]
)


Return('components')
