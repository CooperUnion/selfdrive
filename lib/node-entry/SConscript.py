# ruff: noqa: F821

Import('env')


node_entry = [env.File('entry.c')]


Return('node_entry')
