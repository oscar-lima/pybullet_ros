#!/usr/bin/env python3

"""
Subscribe to cmd_vel and apply a force to a robot, using a PID controller
to match the commanded velocity vs the actual robot speed.
"""

import rospy

from geometry_msgs.msg import Twist
#from simple_pid import PID

import transformations
import numpy as np
import math

class cmdVelCtrl(object):
    def __init__(self, pybullet, robot):
        # get "import pybullet as pb" and store in self.pb
        self.pb = pybullet
        # get robot from parent class
        self.robot = robot
        # subscribe to robot velocity commands
        self.cmd_vel_msg = None
        self.received_cmd_vel_time = None
        rospy.Subscriber("cmd_vel", Twist, self.cmdVelCB)
        # the offset in z direction to where the force will be applied to the model
        self.z_offset = rospy.get_param('~cmd_vel_ctrl/z_offset', -0.35)
        # create 6 controllers, 6 dof (Twist), uncomment for : update_cmd_vel_force()
        # self.controller = []
        # for i in range(0, 6):
            # pid = PID(30000, 0.5, 0.05)
            # pid.sample_time = 0.01
            # pid.output_limits = (-5000, 5000)
            # self.controller.append(pid)

    def update_cmd_vel_force(self):
        """
        experimental code to set a torque on the robot model using a PID controller
        """
        pass
        #self.controller[0].setpoint = self.cmd_vel_msg.linear.x
        #self.controller[1].setpoint = self.cmd_vel_msg.linear.y
        #self.controller[2].setpoint = self.cmd_vel_msg.linear.z
        #self.controller[3].setpoint = self.cmd_vel_msg.angular.x
        #self.controller[4].setpoint = self.cmd_vel_msg.angular.y
        #self.controller[5].setpoint = self.cmd_vel_msg.angular.z
        ## query current robot speed from pybullet
        #linear_vel, angular_vel = self.pb.getBaseVelocity(self.robot)
        ## compute torque value based on PID library
        #output = []
        #for i in range(0, 3):
            #output.append(self.controller[i](linear_vel[i]))
        #for j in range(0, 3):
            #output.append(self.controller[j](angular_vel[j]))
        ## apply external force to robot body
        #self.pb.applyExternalForce(self.robot, linkIndex=-1, forceObj=[output[0], output[1], output[2]],
                                   #posObj=[0.0, 0.0, self.z_offset], flags=self.pb.LINK_FRAME)
        #self.pb.applyExternalTorque(self.robot, linkIndex=-1, torqueObj=[output[3], output[4], output[5]],
                                    #flags=self.pb.LINK_FRAME)

    def update_cmd_vel(self):
        if not self.cmd_vel_msg:
            return
        # check if timestamp is recent

        if (rospy.Time.now() - rospy.Duration(0.5)) > self.received_cmd_vel_time:
            return
        # setpoints
        cmd_vel_lin = [self.cmd_vel_msg.linear.x, self.cmd_vel_msg.linear.y, self.cmd_vel_msg.linear.z, 1.0]
        cmd_vel_ang = [self.cmd_vel_msg.angular.x, self.cmd_vel_msg.angular.y, self.cmd_vel_msg.angular.z, 1.0]
        # == transform vel from robot frame to odom frame, I would use tf.. but has trouble with python3
        # get robot pose from pybullet
        robot_position, robot_orientation_q = self.pb.getBasePositionAndOrientation(self.robot)
        # transform orientation from quat to rpy
        robot_orientation_euler = self.pb.getEulerFromQuaternion(robot_orientation_q)
        # build homogeneous transform using custom transformations library
        m_base_link_to_odom = transformations.compose_matrix(translate=robot_position, angles=robot_orientation_euler)
        cmd_vel_lin_in_odom = np.dot(m_base_link_to_odom, cmd_vel_lin)
        cmd_vel_ang_in_odom = np.dot(m_base_link_to_odom, cmd_vel_ang)
        ## set vel directly on robot model
        self.pb.resetBaseVelocity(self.robot, cmd_vel_lin_in_odom, cmd_vel_ang_in_odom)

    def cmdVelCB(self, msg):
        self.cmd_vel_msg = msg
        self.received_cmd_vel_time = rospy.Time.now()

    def execute(self):
        """this function gets called from pybullet ros main update loop"""
        self.update_cmd_vel()
