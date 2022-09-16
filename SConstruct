#*#*#*#*#*#*#*#*#*#*#*#
#*#*# Cooper IGVC #*#*#
#*#*#*#*#*#*#*#*#*#*#*#

import os

env = Environment()

venv = Virtualenv()
if venv is None:
    print("!!! WARNING: Not running in a virtualenv!")

env.PrependENVPath('PATH', f"{venv}/bin")

Decider('content-timestamp')

term = os.environ.get('TERM') # for color
if term is not None:
    env['ENV']['TERM'] = term

Export('env')

env.SConscript('can/Sconscript', variant_dir='can/build', duplicate=0)
