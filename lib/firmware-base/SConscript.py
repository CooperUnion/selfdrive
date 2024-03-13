# ruff: noqa: F821

Import('env')


# since this has to be compiled uniquely for each micro we'll leave this
# as strings so that we can symlink this directory and turn these into
# File() nodes
firmware_base = [
    f'firmware-base/{src}'
    for src in [
        'app-description.c',
        'eeprom.c',
        'eeprom_ember.c',
        'state-machine.c',
    ]
]


Return('firmware_base')
