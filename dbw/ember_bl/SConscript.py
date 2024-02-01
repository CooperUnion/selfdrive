# ruff: noqa: F821

import os


Import('env')


platformio_ini = env.File('platformio.ini')

ember_bl = env.Phony(
    'ember_bl',
    'platformio run --project-dir ' + os.path.dirname(str(platformio_ini)),
)


Return('ember_bl')
