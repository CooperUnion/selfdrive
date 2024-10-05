# ruff: noqa: F821

Import('env')

drivers = env.SConscript(
    [
        f'{driver}/SConscript.py'
        for driver in [
            'mpu6050',
        ]
    ]
)

Return('drivers')
