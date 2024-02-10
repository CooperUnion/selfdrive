from invoke import (
    call,
    task,
)

from . import direnv


@task(post=[call(direnv.installed, 'CRATES')])
def install(c):
    result = c.run('./tools/cargo-install-gen.py --config \'crates.toml\'')

    for command in result.stdout.strip().split('\n'):
        c.run(command)
