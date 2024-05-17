# ruff: noqa: F821

Import('env')


igvc_report = env.SConscript('igvc-report/SConscript.py')

project, name = env.Project(env.Dir('.'), env.File('project.toml'))
env.ProjectSubtarget(name, 'igvc-report', igvc_report)


Return('project')
