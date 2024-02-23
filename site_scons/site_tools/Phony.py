def Phony(env, target, action):
    return env.AlwaysBuild(env.Alias(target, [], action))


def generate(env):
    if env.Detect('Phony'):
        return

    env.AddMethod(Phony, 'Phony')


def exists(env):
    return env.Detect('Phony')
