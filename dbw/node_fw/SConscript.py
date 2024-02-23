# ruff: noqa: F821

import os


Import('env')


platformio_ini = env.File('platformio.ini')

node_fw = env.Phony(
    'node_fw',
    'platformio run --project-dir ' + os.path.dirname(str(platformio_ini)),
)


Return('node_fw')
