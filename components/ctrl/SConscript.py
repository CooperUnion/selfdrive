# ruff: noqa: F821

Import('env')
Import('envs')


firmware, flash = env.SConscript(
    'src/SConscript.py',
    exports={'env': envs['esp32s3']},
)

component, name = env.Component(firmware, env.File('component.toml'))
env.ComponentSubtarget(name, 'flash', flash)


Return('component')
