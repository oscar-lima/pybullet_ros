#!/usr/bin/env python3

import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'pybullet_ros'

common_files = [f for f in glob('common/**/*', recursive=True) if os.path.isfile(f)]
config_files = [f for f in glob('config/**/*', recursive=True) if os.path.isfile(f)]
launch_files = [f for f in glob('launch/**/*', recursive=True) if os.path.isfile(f)]

setup(
    name=package_name,
    version='2.0.0',
    packages=find_packages(where='pybullet_ros'),
    package_dir={'': 'pybullet_ros'},
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/pybullet_ros']),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', launch_files),
        ('share/' + package_name + '/config', config_files),
        ('share/' + package_name + '/common', common_files),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Oscar Lima',
    maintainer_email='oscar.lima@dfki.de',
    description='ROS wrapper for pybullet simulator',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pybullet_ros_node = pybullet_ros.pybullet_ros:main',
            'joint_states_to_float = pybullet_ros.scripts.joint_states_to_float:main',
        ],
    },
)
