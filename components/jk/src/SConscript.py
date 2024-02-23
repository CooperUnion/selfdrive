# ruff: noqa: F821

Import('env')


main = env.StaticLibrary('main.c')[0]
firmware = env.EspIdf(main, 'esp32s3')


Return('firmware')
