# ruff: noqa: F821

Import('envs')


network = 'network.yml'

for env in envs.values():
    env['CAN'] = {}
    env['CAN']['NETWORK'] = env.File(network)


env = envs['env']


network = env.File(network)
dbc = env.OpenCanDbc(network)
env.Alias('dbc', dbc)


Return('dbc')
