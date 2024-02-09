from invoke import task


@task
def install(c):
    result = c.run('./tools/cargo-install-gen.py --config \'crates.toml\'')

    for command in result.stdout.strip().split('\n'):
        c.run(command)
