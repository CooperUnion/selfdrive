# ruff: noqa: F821

Import('env')


dbc = env.File('network.dbc')
network = env.File('network.yml')

dbc_emmitter = env.Command(
    'create-dbc.py',
    network,
    [
        'opencan-cli compose $SOURCE --dump-python > $TARGET',
        # what a crime
        f'sed -i -e s%opencan\\.dbc%{dbc.path}%g $TARGET',
    ],
)

dbc = env.Command(dbc, dbc_emmitter, 'python $SOURCE')
env.Alias('dbc', dbc)


Return('dbc')
