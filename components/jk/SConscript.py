# ruff: noqa: F821

Import('env')
Import('envs')


firmware = env.SConscript(
    'src/SConscript.py',
    exports={'env': envs['esp32s3']},
)

component = env.Component(firmware, env.File('component.toml'))


Return('component')
