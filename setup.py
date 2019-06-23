#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# for your packages to be recognized by python
d = generate_distutils_setup(
 packages=['pybullet_ros'],
 package_dir={'pybullet_ros': 'ros/src/pybullet_ros'}
)

setup(**d)
