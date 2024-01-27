# ruff: noqa: F821

Import('env')

flags_opt = AddOption(
    '--blpioflags',
    dest='pioflags',
    type='string',
    action='store',
    metavar='-e blink1.1',
    help='PlatformIO environment',
)
env['AddHelp'](
    "ember_bl --blpioflags=FLAGS",
    'Run pio for ember_bl with FLAGS, e.g. `scons ember_bl --blpioflags="run -e blink1.1"`',
)

pioflags = GetOption('pioflags')
command = None
if pioflags is None:
    command = 'pio run -d dbw/ember_bl'
else:
    command = f'pio {pioflags} -d dbw/ember_bl'

[pio_builder] = env.Command(env.Dir('.'), [], command)
env.AlwaysBuild(pio_builder)

env.Alias('ember_bl', pio_builder)
env['AddHelp']('ember_bl', 'Build dbw/ember_bl')
