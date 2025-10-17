#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('pybullet_ros')
    default_config = os.path.join(pkg_share, 'config', 'pybullet_params_example.yaml')
    default_urdf = os.path.join(pkg_share, 'common', 'test', 'urdf', 'r2d2_robot', 'r2d2.urdf.xacro')
    pybullet_launch = os.path.join(pkg_share, 'launch', 'pybullet_ros_example.launch.py')

    robot_description = ParameterValue(
        Command(['xacro', LaunchConfiguration('robot_urdf_path')]),
        value_type=str,
    )

    return LaunchDescription([
        DeclareLaunchArgument('config_file', default_value=default_config,
                              description='YAML configuration file for pybullet_ros'),
        DeclareLaunchArgument('plugin_import_prefix', default_value='pybullet_ros.plugins',
                              description='Python module prefix used for plugin imports'),
        DeclareLaunchArgument('environment', default_value='environment',
                              description='Environment plugin name'),
        DeclareLaunchArgument('pybullet_gui', default_value='true',
                              description='Enable pybullet GUI'),
        DeclareLaunchArgument('robot_urdf_path', default_value=default_urdf,
                              description='Robot URDF or xacro file to load'),
        DeclareLaunchArgument('pause_simulation', default_value='false',
                              description='Start simulation paused'),
        DeclareLaunchArgument('parallel_plugin_execution', default_value='true',
                              description='Execute plugins in parallel'),
        DeclareLaunchArgument('robot_pose_x', default_value='0.0'),
        DeclareLaunchArgument('robot_pose_y', default_value='0.0'),
        DeclareLaunchArgument('robot_pose_z', default_value='0.7'),
        DeclareLaunchArgument('robot_pose_yaw', default_value='0.0'),
        DeclareLaunchArgument('fixed_base', default_value='false'),
        DeclareLaunchArgument('use_deformable_world', default_value='false'),
        DeclareLaunchArgument('gui_options', default_value=''),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(pybullet_launch),
            launch_arguments={
                'config_file': LaunchConfiguration('config_file'),
                'plugin_import_prefix': LaunchConfiguration('plugin_import_prefix'),
                'environment': LaunchConfiguration('environment'),
                'pybullet_gui': LaunchConfiguration('pybullet_gui'),
                'robot_urdf_path': LaunchConfiguration('robot_urdf_path'),
                'pause_simulation': LaunchConfiguration('pause_simulation'),
                'parallel_plugin_execution': LaunchConfiguration('parallel_plugin_execution'),
                'robot_pose_x': LaunchConfiguration('robot_pose_x'),
                'robot_pose_y': LaunchConfiguration('robot_pose_y'),
                'robot_pose_z': LaunchConfiguration('robot_pose_z'),
                'robot_pose_yaw': LaunchConfiguration('robot_pose_yaw'),
                'fixed_base': LaunchConfiguration('fixed_base'),
                'use_deformable_world': LaunchConfiguration('use_deformable_world'),
                'gui_options': LaunchConfiguration('gui_options'),
            }.items(),
        ),
    ])
