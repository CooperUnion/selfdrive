# ruff: noqa: F821

Import('env')

source = [env['LIBRARIES']['drivers']]


main = env.StaticLibrary('main', ['main.c', source])[0]
firmware = env.EspIdf(main, 'esp32s3')


Return('firmware')
