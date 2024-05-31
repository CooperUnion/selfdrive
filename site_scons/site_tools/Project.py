import tomllib

from schema import (
    Regex,
    Schema,
)


schema = Schema(
    {
        'metadata': {
            'name': Regex(r'^[a-z-]+$'),
        },
    }
)


def Project(env, target, config):
    with open(config.srcnode().abspath, 'rb') as f:
        config = tomllib.load(f)

    assert schema.validate(config) == config

    name = config['metadata']['name']

    env['PROJECTS'][name] = {
        'subtargets': {},
        'target': target,
    }

    return env.Alias(f'project:{name}', target, ''), name


def ProjectSubtarget(env, project, subtarget, target):
    env['PROJECTS'][project]['subtargets'][subtarget] = target

    return env.Alias(f'project:{project}:{subtarget}', target, '')


def generate(env):
    if env.Detect('Project') and env.Detect('ProjectSubtarget'):
        return

    env.AddMethod(Project, 'Project')
    env.AddMethod(ProjectSubtarget, 'ProjectSubtarget')

    env['PROJECTS'] = {}


def exists(env):
    return env.Detect('Project') and env.Detect('ProjectSubtarget')
