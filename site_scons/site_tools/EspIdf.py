import os


OUTPUTS = [
    'bootloader/config/sdkconfig.h',
    'bootloader/.bin_timestamp',
    'bootloader/bootloader.bin',
    'bootloader/bootloader.elf',
    'bootloader/bootloader.map',
    'bootloader/prefix_map_gdbinit',
    'config/sdkconfig.h',
    'partition_table/partition-table.bin',
    '.bin_timestamp',
    'app-flash_args',
    'bootloader-flash_args',
    'firmware.bin',
    'firmware.elf',
    'firmware.map',
    'flash_app_args',
    'flash_args',
    'flash_bootloader_args',
    'flash_project_args',
    'flasher_args.json',
    'ldgen_libraries',
    'ota_data_initial.bin',
    'partition-table-flash_args',
    'prefix_map_gdbinit',
    'x509_crt_bundle.S',
]


def EspIdf(env, library, target, *, outdir='esp-idf'):
    build = f'{os.environ["IDF_BUILD"]}/march/{target}'
    outdir = f'{os.path.dirname(library.abspath)}/{outdir}'

    libprebuilt = (
        f'{os.environ["REPO_ROOT"]}/lib/march/{target}/main/libprebuilt.a'
    )

    # since the esp-idf build is shared resource we need to
    # lock it while we compile and link with out shared library
    # we also need to remove the generated elfs for when the
    # timestamp isn't old enough for competing threads
    actions = [
        f'exec {{LOCKFD}}> {build}/scons.lock',
        'flock --exclusive $$LOCKFD',
        f'rm -f {build}/bootloader/bootloader.elf',
        f'rm -f {build}/firmware.elf',
        f'rm -f {libprebuilt}',
        f'ln -s {library.abspath} {libprebuilt}',
        f'ninja -C {build}',
        f'mkdir -p {outdir}/bootloader',
        f'mkdir -p {outdir}/partition_table',
        *[f'cp {build}/{output} {outdir}/{output}' for output in OUTPUTS],
        'exec {LOCKFD}>&-',
    ]

    actions = ' && '.join(actions)

    if not env['VERBOSE']:
        actions = f'@echo ESP-IDF {library.path} && ' + actions

    out = env.Command(
        [f'{outdir}/{output}' for output in OUTPUTS], library, actions
    )

    flash = EspTool(
        env=env,
        dir=outdir,
        chip=target,
        args=['write_flash', '@flash_args'],
    )
    env.Depends(flash, out)

    return out, flash


def EspTool(env, dir, chip, args):
    esptool_args = [f'--chip {chip}']

    if port := env['ESPPORT']:
        esptool_args += [f'--port {port}']

    if baud := env['ESPBAUD']:
        esptool_args += [f'--baud {baud}']

    esptool_args += args

    esptool_args = ' '.join(esptool_args)

    actions = [
        f'cd {dir}',
        f'esptool.py {esptool_args}',
    ]

    actions = ' && '.join(actions)

    out = env.Phony(f'{dir}:flash', actions)

    return out


def generate(env):
    if env.Detect('EspIdf') and env.Detect('EspTool'):
        return

    env.AddMethod(EspIdf, 'EspIdf')
    env.AddMethod(EspTool, 'EspTool')


def exists(env):
    return env.Detect('EspIdf') and env.Detect('EspTool')
