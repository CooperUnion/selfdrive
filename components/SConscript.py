# ruff: noqa: F821

Import('env')


components = env.SConscript(
    [
        f'{component}/SConscript.py'
        for component in [
            'bbc',
            'gas',
            'jk',
            'om',
            'skrt',
        ]
    ]
)


Return('components')
