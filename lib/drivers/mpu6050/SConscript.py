# ruff: noqa: F821

Import('env')

source = [env.File('mpu6050.c'), env.File('i2c_hal_esp32.c')]

Return('source')
