#!/usr/bin/python3

# from distutils.core import setup
from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['drone_wrapper'],
    package_dir={'': 'src'}
)
setup(**d)
