# ruff: noqa: F821

Import('env')


projects = env.SConscript(
    [
        f'{project}/SConscript.py'
        for project in [
            'g-wagon',
        ]
    ]
)


Return('projects')
