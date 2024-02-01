# ruff: noqa: F821

import os


Import('env')


platformio_ini = env.File('platformio.ini')

ember_bl = env.Command(
    '.ember_bl',
    platformio_ini,
    'platformio run --project-dir ' + os.path.dirname(str(platformio_ini)),
)
env.AlwaysBuild(ember_bl)
env.Alias('ember_bl', ember_bl)


Return('ember_bl')
