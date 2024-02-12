from invoke import (
    call,
    task,
)

from . import direnv


@task(post=[call(direnv.installed, 'PYTHON_REQUIREMENTS')])
def install_requirements(c):
    c.run('pip install --requirement requirements.txt')
    c.run('pip install --requirement requirements-local.txt')


@task
def pip_compile(c):
    c.run('pip-compile --output-file requirements.txt')


@task
def venv(c):
    c.run('python -m venv .venv')
