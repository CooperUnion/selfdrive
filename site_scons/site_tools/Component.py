import tomllib

from schema import (
    Regex,
    Schema,
)


schema = Schema(
    {
        'metadata': {
            'name': Regex(r'^[a-z]+$'),
            'description': str,
        },
    }
)


def Component(env, target, config):
    with open(config.srcnode().abspath, 'rb') as f:
        config = tomllib.load(f)

    assert schema.validate(config) == config

    name = config['metadata']['name']

    return env.Phony(f'component:{name}', target)


def generate(env):
    if env.Detect('Component'):
        return

    env.AddMethod(Component, 'Component')


def exists(env):
    return env.Detect('Component')
