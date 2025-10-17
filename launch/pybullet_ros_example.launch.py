#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('pybullet_ros')
    default_config = os.path.join(pkg_share, 'config', 'pybullet_params_example.yaml')
    default_urdf = os.path.join(pkg_share, 'common', 'test', 'urdf', 'r2d2_robot', 'r2d2.urdf')

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
                              description='Robot URDF file to load'),
        DeclareLaunchArgument('pause_simulation', default_value='false',
                              description='Start simulation paused'),
        DeclareLaunchArgument('parallel_plugin_execution', default_value='true',
                              description='Execute plugins in parallel'),
        DeclareLaunchArgument('robot_pose_x', default_value='0.0',
                              description='Robot spawn position X'),
        DeclareLaunchArgument('robot_pose_y', default_value='0.0',
                              description='Robot spawn position Y'),
        DeclareLaunchArgument('robot_pose_z', default_value='1.0',
                              description='Robot spawn position Z'),
        DeclareLaunchArgument('robot_pose_yaw', default_value='0.0',
                              description='Robot spawn yaw'),
        DeclareLaunchArgument('fixed_base', default_value='false',
                              description='Spawn robot with fixed base'),
        DeclareLaunchArgument('use_deformable_world', default_value='false',
                              description='Enable pybullet soft body simulation'),
        DeclareLaunchArgument('gui_options', default_value='',
                              description='Additional GUI options passed to pybullet'),
        Node(
            package='pybullet_ros',
            executable='pybullet_ros_node',
            name='pybullet_ros',
            output='screen',
            parameters=[
                {
                    'config_file': LaunchConfiguration('config_file'),
                    'plugin_import_prefix': LaunchConfiguration('plugin_import_prefix'),
                    'environment': LaunchConfiguration('environment'),
                    'pybullet_gui': ParameterValue(LaunchConfiguration('pybullet_gui'), value_type=bool),
                    'robot_urdf_path': LaunchConfiguration('robot_urdf_path'),
                    'pause_simulation': ParameterValue(LaunchConfiguration('pause_simulation'), value_type=bool),
                    'parallel_plugin_execution': ParameterValue(
                        LaunchConfiguration('parallel_plugin_execution'), value_type=bool
                    ),
                    'robot_pose_x': ParameterValue(LaunchConfiguration('robot_pose_x'), value_type=float),
                    'robot_pose_y': ParameterValue(LaunchConfiguration('robot_pose_y'), value_type=float),
                    'robot_pose_z': ParameterValue(LaunchConfiguration('robot_pose_z'), value_type=float),
                    'robot_pose_yaw': ParameterValue(LaunchConfiguration('robot_pose_yaw'), value_type=float),
                    'fixed_base': ParameterValue(LaunchConfiguration('fixed_base'), value_type=bool),
                    'use_deformable_world': ParameterValue(
                        LaunchConfiguration('use_deformable_world'), value_type=bool
                    ),
                    'gui_options': LaunchConfiguration('gui_options'),
                }
            ],
        ),
    ])
