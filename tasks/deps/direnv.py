import os

from invoke import task


@task
def installed(c, var):
    path = os.environ[f'DIRENV_INSTALLED_{var}']
    c.run(f'touch "{path}"')
