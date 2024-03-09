# ruff: noqa: F821

Import('env')


node_entry = env.StaticLibrary('node-entry', 'entry.c')


Return('node_entry')
