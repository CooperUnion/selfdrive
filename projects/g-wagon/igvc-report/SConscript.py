# ruff: noqa: F821

Import('env')


igvc_report = env.PDF('igvc-report.tex')


Return('igvc_report')
