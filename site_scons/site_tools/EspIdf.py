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
    actions = [
        f'exec {{LOCKFD}}> {build}/scons.lock',
        'echo $$LOCKFD',
        'flock --exclusive $$LOCKFD',
        f'rm -f {libprebuilt}',
        f'ln -s {library.abspath} {libprebuilt}',
        f'ninja -C {build}',
        f'mkdir -p {outdir}/bootloader',
        f'mkdir -p {outdir}/partition_table',
        *[f'cp {build}/{output} {outdir}/{output}' for output in OUTPUTS],
        'exec {LOCKFD}>&-',
    ]

    actions = ' && '.join(actions)

    out = env.Command(
        [f'{outdir}/{output}' for output in OUTPUTS], library, actions
    )

    return out


def generate(env):
    if env.Detect('EspIdf'):
        return

    env.AddMethod(EspIdf, 'EspIdf')


def exists(env):
    return env.Detect('EspIdf')
