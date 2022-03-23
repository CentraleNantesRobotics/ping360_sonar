#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
     packages=['ping360_sonar'],
     package_dir={'': 'src'},
     install_requires=['bluerobotics-ping']
)

setup(**setup_args)
