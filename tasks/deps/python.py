from invoke import task


@task
def install_requirements(c):
    c.run('pip install --requirement requirements.txt')
    c.run('pip install --requirement requirements-local.txt')
