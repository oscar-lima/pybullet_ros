#!/usr/bin/env python3

"""
Subscribe to cmd_vel and apply desired speed to the robot, without any noise

tf explained:
pybullet requires that velocity of the robot is set w.r.t. world reference frame
however cmd_vel convention required velocity to be expressed w.r.t. robot base frame
therefore a transformation is needed.
"""

import rospy
import math
import numpy as np

from geometry_msgs.msg import Twist, Vector3Stamped, Vector3

class cmdVelCtrl:
    def __init__(self, pybullet, robot, **kargs):
        # get "import pybullet as pb" and store in self.pb
        self.pb = pybullet
        # get robot from parent class
        self.robot = robot
        # subscribe to robot velocity commands
        self.cmd_vel_msg = None
        self.received_cmd_vel_time = None
        rospy.Subscriber("cmd_vel", Twist, self.cmdVelCB)

    # ---------- tf stuff starts

    def translation_matrix(self, direction):
        """copied from tf (listener.py)"""
        M = np.identity(4)
        M[:3, 3] = direction[:3]
        return M

    def quaternion_matrix(self, quaternion):
        """copied from tf (listener.py)"""
        epsilon = np.finfo(float).eps * 4.0
        q = np.array(quaternion[:4], dtype=np.float64, copy=True)
        nq = np.dot(q, q)
        if nq < epsilon:
            return np.identity(4)
        q *= math.sqrt(2.0 / nq)
        q = np.outer(q, q)
        return np.array((
            (1.0-q[1, 1]-q[2, 2],     q[0, 1]-q[2, 3],     q[0, 2]+q[1, 3], 0.0),
            (    q[0, 1]+q[2, 3], 1.0-q[0, 0]-q[2, 2],     q[1, 2]-q[0, 3], 0.0),
            (    q[0, 2]-q[1, 3],     q[1, 2]+q[0, 3], 1.0-q[0, 0]-q[1, 1], 0.0),
            (                0.0,                 0.0,                 0.0, 1.0)
            ), dtype=np.float64)

    def fromTranslationRotation(self, translation, rotation):
        """copied from tf (listener.py)"""
        return np.dot(self.translation_matrix(translation), self.quaternion_matrix(rotation))

    def lookupTransform(self, target_frame='base_link', source_frame='odom'):
        """
        copied from tf (listener.py)
        source_frame = odom, world_frame, target_frame = base_link, robot_frame
        """
        # get robot pose from pybullet
        t, r = self.pb.getBasePositionAndOrientation(self.robot)
        return [t[0], t[1], t[2]], [r[0], r[1], r[2], r[3]]

    def asMatrix(self, target_frame, hdr):
        """copied from tf (listener.py)"""
        translation,rotation = self.lookupTransform(target_frame, hdr.frame_id)
        return self.fromTranslationRotation(translation, rotation)

    def transformVector3(self, target_frame, v3s):
        """copied from tf (listener.py)"""
        mat44 = self.asMatrix(target_frame, v3s.header)
        mat44[0,3] = 0.0
        mat44[1,3] = 0.0
        mat44[2,3] = 0.0
        xyz = tuple(np.dot(mat44, np.array([v3s.vector.x, v3s.vector.y, v3s.vector.z, 1.0])))[:3]
        r = Vector3Stamped()
        r.header.stamp = v3s.header.stamp
        r.header.frame_id = target_frame
        r.vector = Vector3(*xyz)
        return r

    # ---------- tf stuff ends

    def cmd_vel_force_ctrler(self):
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
        #self.z_offset = rospy.get_param('~cmd_vel_ctrl/z_offset', -0.35)
        #self.pb.applyExternalForce(self.robot, linkIndex=-1, forceObj=[output[0], output[1], output[2]],
                                    #posObj=[0.0, 0.0, self.z_offset], flags=self.pb.LINK_FRAME)
        #self.pb.applyExternalTorque(self.robot, linkIndex=-1, torqueObj=[output[3], output[4], output[5]],
                                    #flags=self.pb.LINK_FRAME)

    def cmdVelCB(self, msg):
        """callback to receive vel commands from user"""
        self.cmd_vel_msg = msg
        self.received_cmd_vel_time = rospy.Time.now()

    def execute(self):
        """this function gets called from pybullet ros main update loop"""
        if not self.cmd_vel_msg:
            return
        # check if timestamp is recent
        if (rospy.Time.now() - rospy.Duration(0.5)) > self.received_cmd_vel_time:
            return
        # transform Twist from base_link to odom (pybullet allows to set vel only on world ref frame)
        # NOTE: we would normally use tf for this, but there are issues currently between python 2 and 3 in ROS 1
        lin_vec = Vector3Stamped()
        lin_vec.header.frame_id = 'base_link'
        lin_vec.vector.x = self.cmd_vel_msg.linear.x
        lin_vec.vector.y = self.cmd_vel_msg.linear.y
        lin_vec.vector.z = self.cmd_vel_msg.linear.z
        ang_vec = Vector3Stamped()
        ang_vec.header.frame_id = 'base_link'
        ang_vec.vector.x = self.cmd_vel_msg.angular.x
        ang_vec.vector.y = self.cmd_vel_msg.angular.y
        ang_vec.vector.z = self.cmd_vel_msg.angular.z
        lin_cmd_vel_in_odom = self.transformVector3('odom', lin_vec)
        ang_cmd_vel_in_odom = self.transformVector3('odom', ang_vec)
        # set vel directly on robot model
        self.pb.resetBaseVelocity(self.robot, [lin_cmd_vel_in_odom.vector.x, lin_cmd_vel_in_odom.vector.y, lin_cmd_vel_in_odom.vector.z],
                                  [ang_cmd_vel_in_odom.vector.x, ang_cmd_vel_in_odom.vector.y, ang_cmd_vel_in_odom.vector.z])
