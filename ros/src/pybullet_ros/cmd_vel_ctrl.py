#!/usr/bin/env python3

"""
Subscribe to cmd_vel and apply a force to a robot, using a PID controller
to match the commanded velocity vs the actual robot speed.
"""

import rospy

from geometry_msgs.msg import Twist
from simple_pid import PID

class cmdVelCtrl(object):
    def __init__(self, pybullet, robot):
        # get "import pybullet as pb" and store in self.pb
        self.pb = pybullet
        # get robot from parent class
        self.robot = robot
        rospy.Subscriber("cmd_vel", Twist, self.cmdVelCB)
        self.cmd_vel_msg = None
        # cmd vel control for vx
        self.pid = PID(30000, 0.5, 0.05)
        self.pid.sample_time = 0.01
        self.pid.output_limits = (-5000, 5000)

    def update_cmd_vel(self):
        if not self.cmd_vel_msg:
            return
        # P gain, I gain, D gain, setpoint
        self.pid.setpoint = self.cmd_vel_msg.linear.x
        # remove
        rospy.loginfo('setpoint : %s', self.pid.setpoint)
        # query current x speed from pybullet
        linear_vel, angular_vel = self.pb.getBaseVelocity(self.robot)
        current_value = linear_vel[0]
        # remove
        rospy.loginfo('current value : %s', current_value)
        # compute torque value
        output = self.pid(current_value)
        # remove
        rospy.loginfo('command : %s', output)
        rospy.loginfo('------------')
        # send linear speed command
        z_offset = rospy.get_param('~cmd_vel_ctrl/z_offset', -0.35)
        # apply external force to robot base
        self.pb.applyExternalForce(self.robot, linkIndex=-1, forceObj=[output, 0.0, 0.0], posObj=[0.0, 0.0, z_offset], flags=self.pb.LINK_FRAME)

    def cmdVelCB(self, msg):
        self.cmd_vel_msg = msg

    def execute(self):
        """this function gets called from pybullet ros main update loop"""
        self.update_cmd_vel()
