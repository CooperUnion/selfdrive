#*#*#*#*#*#*#*#*#*#*#*#
#*#*# Cooper IGVC #*#*#
#*#*#*#*#*#*#*#*#*#*#*#

import os

# We should get more disciplined about our envs later.
env = Environment(ENV = {'PATH' : os.environ['PATH']})

# Save the repo root in the env
env['REPO_ROOT'] = env.Dir('.')

Decider('content-timestamp')

term = os.environ.get('TERM') # for color
if term is not None:
    env['ENV']['TERM'] = term

Default(None)
Export('env')

# Dependencies first
env.SConscript('dependencies.SConscript', variant_dir='deps', duplicate=0)

env.SConscript('can/SConscript', variant_dir='build/can', duplicate=0)
