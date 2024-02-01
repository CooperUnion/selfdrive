# ruff: noqa: F821

import os


Import('env')


platformio_ini = env.File('platformio.ini')

node_fw = env.Command(
    '.node_fw',
    platformio_ini,
    'platformio run --project-dir ' + os.path.dirname(str(platformio_ini)),
)
env.AlwaysBuild(node_fw)
env.Alias('node_fw', node_fw)


Return('node_fw')
