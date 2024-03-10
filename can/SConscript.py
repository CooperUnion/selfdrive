# ruff: noqa: F821

Import('env')


network = env.File('network.yml')
dbc = env.OpenCanDbc(network)
env.Alias('dbc', dbc)


Return('dbc')
