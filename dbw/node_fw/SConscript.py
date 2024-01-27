# ruff: noqa: F821

Import('env')

flags_opt = AddOption(
    '--pioflags',
    dest='pioflags',
    type='string',
    action='store',
    metavar='-e blink1.1',
    help='PlatformIO environment',
)
env['AddHelp'](
    "node_fw --pioflags=FLAGS",
    'Run pio for node_fw with FLAGS, e.g. `scons node_fw --pioflags="run -e blink1.1"`',
)

pioflags = GetOption('pioflags')
command = None
if pioflags is None:
    command = 'pio run -d dbw/node_fw'
else:
    command = f'pio {pioflags} -d dbw/node_fw'

[pio_builder] = env.Command(env.Dir('.'), [], command)
env.AlwaysBuild(pio_builder)

env.Alias('node_fw', pio_builder)
env['AddHelp']('node_fw', 'Build dbw/node_fw')
