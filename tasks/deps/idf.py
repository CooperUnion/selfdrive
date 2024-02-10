import json
import os
import shlex

from invoke import task


@task
def install(c):
    c.run(f'{os.environ["IDF_PATH"]}/install.sh "{os.environ["IDF_TARGETS"]}"')


@task
def scons_env_gen(c):
    # forgive me lord, for i have sinned
    # esp-idf really doesn't like multi-binary builds...
    # to get around this, we instead link a static object with a shared
    # installation of esp-idf, however, scons needs the following information
    # generated so that the experience is seamless

    prefix = f'. {os.environ["IDF_PATH"]}/export.sh'

    with c.prefix(prefix):
        # we insert a \r to allow us to ignore export.sh output
        result = c.run('echo -e \'\\r\'"$PATH"')

    path = result.stdout.strip().split('\r')[-1]

    # keep only the new paths
    idf_path = list(set(path.split(':')) ^ set(result.env['PATH'].split(':')))

    for target in os.environ['IDF_TARGETS'].split(','):
        build = f'{os.environ["IDF_BUILD"]}/march/{target}'
        src = f'lib/march/{target}'

        c.run(
            f'cmake -B "{build}" -S "{src}"',
            env={
                'IDF_TARGET': target,
                'PATH': path,
            },
        )

        with open(f'{build}/compile_commands.json') as f:
            compile_commands = json.load(f)

        main_c = f'{src}/main/main.c'
        main = list(
            filter(lambda x: x['file'].endswith(main_c), compile_commands)
        )[0]

        flags = shlex.split(main['command'])

        def startswith(x):
            return lambda y: y.startswith(x)

        includes = list(filter(startswith('-I'), flags))
        includes = [include.removeprefix('-I') for include in includes]

        options = list(filter(startswith('-f'), flags))
        options.remove('-fdiagnostics-color=always')

        defines = list(filter(startswith('-D'), flags))
        optimiation = list(filter(startswith('-O'), flags))
        debug = list(filter(startswith('-g'), flags))
        march = list(filter(startswith('-m'), flags))
        std = list(filter(startswith('-std'), flags))

        cflags = options + defines + optimiation + debug + march + std

        env = {
            'prefix': f'xtensa-{target}-elf-',
            'cflags': cflags,
            'includes': includes,
            'path': idf_path,
        }

        env_file = os.environ['IDF_BUILD_SCONS_ESP32S3']

        os.makedirs(os.path.dirname(env_file), exist_ok=True)
        with open(env_file, 'w') as f:
            json.dump(env, f, indent=4)