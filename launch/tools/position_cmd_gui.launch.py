#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='position_ctrl_cmd_gui',
            remappings=[('joint_states', 'fake_joint_states')],
        ),
        Node(
            package='pybullet_ros',
            executable='joint_states_to_float',
            name='position_ctrl_repub',
            output='screen',
        ),
    ])
