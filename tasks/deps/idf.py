import os

from invoke import task


@task
def install(c):
    c.run(f'{os.environ["IDF_PATH"]}/install.sh "{os.environ["IDF_TARGETS"]}"')
