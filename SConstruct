#*#*#*#*#*#*#*#*#*#*#*#
#*#*# Cooper IGVC #*#*#
#*#*#*#*#*#*#*#*#*#*#*#

import os

# Basic setup ---------------------------------------------
# We should get more disciplined about our PATH later.
env = Environment(ENV = {'PATH' : os.environ['PATH']})

# Save the repo root in the env
env['REPO_ROOT'] = env.Dir('.')

Decider('content-timestamp')

term = os.environ.get('TERM') # for color
if term is not None:
    env['ENV']['TERM'] = term
# ---------------------------------------------------------


# Global help adder function ------------------------------
help_list = []

def AddHelp(cmd, text):
    global help_list
    help_list.append((cmd, text))

env['AddHelp'] = AddHelp
# ---------------------------------------------------------


# Cleaning targets ----------------------------------------
[rm_build] = env.Command(
    'phony-rm-build',
    [],
    'rm -rf build/'
)

[rm_deps] = env.Command(
    'phony-rm-deps',
    [],
    'rm -rf deps/'
)

env.Alias('clean',    rm_build)
env.Alias('cleanall', [rm_build, rm_deps])
AddHelp('clean',    'Clean (remove) build/ directory')
AddHelp('cleanall', 'Clean (remove) build/ and deps/ (aka everything)')
# ---------------------------------------------------------


# Call SConscripts ----------------------------------------
Default(None)
Export('env')

# Dependencies first
env.SConscript('dependencies.SConscript', variant_dir='deps',              duplicate=0)
env.SConscript('can/SConscript',          variant_dir='build/can',         duplicate=0)
env.SConscript('dbw/node_fw/SConscript',  variant_dir='build/dbw/node_fw', duplicate=0)
# ---------------------------------------------------------


# Populate Help -------------------------------------------
# scons provides Help for you to call to provide the text given by `scons -h`.
# you can call Help more than once and it will append.
Help('''
     So you want to build a car?

     You can specify targets after `scons`, like:

''')

help_list.sort()

for (cmd, text) in help_list:
    Help(f"     `scons {cmd + '`' : <30} {text : <60}\n")
# ---------------------------------------------------------

if not COMMAND_LINE_TARGETS:
    from SCons.Script import help_text
    print(help_text)
    exit(0)
