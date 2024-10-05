# ruff: noqa: F821

Import('env')

env.AppendUnique(CPPPATH=env.Dir('.'))

source = [env.File('mpu6050.c')]

Return('source')
