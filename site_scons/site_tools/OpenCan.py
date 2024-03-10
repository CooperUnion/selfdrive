import os


OUTPUTS = [
    f'opencan_{file}'
    for file in [
        'callbacks.h',
        'rx.c',
        'rx.h',
        'templates.h',
        'tx.c',
        'tx.h',
    ]
]


def OpenCan(env, network, node, *, GEN_DIR=None):
    if GEN_DIR is None:
        GEN_DIR = env.Dir('opencan-codegen')

    out = env.Command(
        [f'{GEN_DIR.path}/{output}' for output in OUTPUTS],
        network,
        f'opencan-cli codegen {network.path} {GEN_DIR.path} {node}',
    )

    return out


def OpenCanDbc(env, network):
    dbc = env.File(os.path.splitext(network.path)[0] + '.dbc')

    dbc_emmitter = env.Command(
        'dbc-emmitter.py',
        network,
        [
            'opencan-cli compose $SOURCE --dump-python > $TARGET',
            # what a crime
            f'sed -i -e s%opencan\\.dbc%{dbc.path}%g $TARGET',
        ],
    )

    dbc = env.Command(dbc, dbc_emmitter, 'python $SOURCE')

    return dbc


def generate(env):
    if env.Detect('OpenCan') and env.Detect('OpenCanDbc'):
        return

    env.AddMethod(OpenCan, 'OpenCan')
    env.AddMethod(OpenCanDbc, 'OpenCanDbc')


def exists(env):
    return env.Detect('OpenCan') and env.Detect('OpenCanDbc')
