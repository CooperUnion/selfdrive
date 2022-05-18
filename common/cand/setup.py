#!/usr/bin/env python3

import setuptools


with open("README.md", "r") as fh:
    long_description = fh.read()

setuptools.setup(
    name="cand",
    version="0.0.2",
    author="Cooper IGVC",
    author_email="igvc@cooper.edu",
    description="CAN Service Daemon",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://git.cooperigvc.org/igvc/selfdrive/-/tree/master/common/cand",
    packages=["cand"],
    install_requires=[
        "cantools>=37.0.7",
        "coloredlogs>=15.0.1",
        "msgpack>=1.0.3",
        "python-can>=3.3.4",
        "redis>=4.2.2",
    ],
    python_requires=">=3.8",
    scripts=["cand/cand"],
)
