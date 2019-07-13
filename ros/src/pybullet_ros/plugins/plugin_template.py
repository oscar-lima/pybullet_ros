#!/usr/bin/env python3

"""
TODO: briefly describe your plugin here
"""

import rospy

class pluginTemplate:
    def __init__(self, pybullet, robot, **kargs):
        # get "import pybullet as pb" and store in self.pb
        self.pb = pybullet
        # get robot from parent class
        self.robot = robot
        # TODO: implement here...

    def execute(self):
        """this function gets called from pybullet ros main update loop"""
        rospy.loginfo('my plugin is running!')
