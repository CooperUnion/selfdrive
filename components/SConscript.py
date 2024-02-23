# ruff: noqa: F821

Import('env')


components = []

components += env.SConscript(
    'jk/SConscript.py',
)


Return('components')
