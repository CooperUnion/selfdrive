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

    env['COMPONENTS'][name] = {
        'subtargets': {},
        'target': target,
    }

    return env.Alias(f'component:{name}', target, ''), name


def ComponentSubtarget(env, component, subtarget, target):
    env['COMPONENTS'][component]['subtargets'][subtarget] = target

    return env.Alias(f'component:{component}:{subtarget}', target, '')


def generate(env):
    if env.Detect('Component') and env.Detect('ComponentSubtarget'):
        return

    env.AddMethod(Component, 'Component')
    env.AddMethod(ComponentSubtarget, 'ComponentSubtarget')

    env['COMPONENTS'] = {}


def exists(env):
    return env.Detect('Component') and env.Detect('ComponentSubtarget')
