#!/usr/bin/env python3

import setuptools


with open('README.md', 'r') as fh:
    long_description = fh.read()

setuptools.setup(
    name='igvcutils',
    version='0.0.4',
    author='Cooper IGVC',
    author_email='igvc@cooper.edu',
    description='IGVC utilities',
    long_description=long_description,
    long_description_content_type='text/markdown',
    url='https://github.com/CooperUnion/selfdrive/tree/master/common/igvcutils',
    package_dir={'': 'src'},
    packages=setuptools.find_packages(where='src'),
    install_requires=[
        'cantools>=37.0.7',
        'python-can>=3.3.4',
    ],
    python_requires='>=3.9',
)
