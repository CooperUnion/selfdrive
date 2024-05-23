# ruff: noqa: F821

Import('env')


env.AppendUnique(CPPPATH=env.Dir('include'))

selfdrive = env.SConscript('src/SConscript.py')


Return('selfdrive')
