#!/usr/bin/env python3

"""Example structure for a pybullet_ros plugin."""


class PluginTemplate:
    def __init__(self, node, pybullet, robot, **kwargs):
        self.node = node
        self.logger = node.get_logger()
        self.pb = pybullet
        self.robot = robot
        # TODO: implement plugin initialisation here

    def execute(self):
        """Called from the pybullet ros update loop."""
        self.logger.info('My plugin is running!')


# Backwards compatibility alias
pluginTemplate = PluginTemplate
