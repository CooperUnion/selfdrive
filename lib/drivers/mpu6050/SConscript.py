# ruff: noqa: F821

Import('env')

source = [env.File('mpu6050.c')]

Return('source')
