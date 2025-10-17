#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_share = get_package_share_directory('pybullet_ros')
    default_config = os.path.join(pkg_share, 'config', 'robots', 'acrobat_robot_config.yaml')
    default_urdf = os.path.join(pkg_share, 'common', 'test', 'urdf', 'acrobat_robot', 'acrobat_robot.urdf')
    bringup_launch = os.path.join(pkg_share, 'launch', 'bringup_robot_example.launch.py')
    tools_launch = os.path.join(pkg_share, 'launch', 'tools', 'position_cmd_gui.launch.py')

    return LaunchDescription([
        DeclareLaunchArgument('config_file', default_value=default_config,
                              description='Configuration for the acrobat robot'),
        DeclareLaunchArgument('robot_urdf_path', default_value=default_urdf,
                              description='URDF file for the acrobat robot'),
        DeclareLaunchArgument('robot_pose_z', default_value='0.0'),
        DeclareLaunchArgument('fixed_base', default_value='true'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(bringup_launch),
            launch_arguments={
                'config_file': LaunchConfiguration('config_file'),
                'robot_urdf_path': LaunchConfiguration('robot_urdf_path'),
                'robot_pose_z': LaunchConfiguration('robot_pose_z'),
                'fixed_base': LaunchConfiguration('fixed_base'),
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(tools_launch),
        ),
    ])
