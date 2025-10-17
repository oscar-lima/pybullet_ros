#!/usr/bin/env python3

"""Subscribe to cmd_vel and apply desired speed to the robot without any noise."""

import math
from typing import Optional

import numpy as np
from geometry_msgs.msg import Twist, Vector3, Vector3Stamped
from rclpy.duration import Duration


class CmdVelCtrl:
    def __init__(self, node, pybullet, robot, **kwargs):
        self.node = node
        self.logger = node.get_logger()
        self.pb = pybullet
        self.robot = robot
        self.cmd_vel_msg: Optional[Twist] = None
        self.received_cmd_vel_time = None

        topic = kwargs.get('parameters', {}).get('cmd_vel', 'cmd_vel')
        self.subscription = node.create_subscription(Twist, topic, self.cmd_vel_callback, 10)

    def translation_matrix(self, direction):
        M = np.identity(4)
        M[:3, 3] = direction[:3]
        return M

    def quaternion_matrix(self, quaternion):
        epsilon = np.finfo(float).eps * 4.0
        q = np.array(quaternion[:4], dtype=np.float64, copy=True)
        nq = np.dot(q, q)
        if nq < epsilon:
            return np.identity(4)
        q *= math.sqrt(2.0 / nq)
        q = np.outer(q, q)
        return np.array(
            (
                (1.0 - q[1, 1] - q[2, 2], q[0, 1] - q[2, 3], q[0, 2] + q[1, 3], 0.0),
                (q[0, 1] + q[2, 3], 1.0 - q[0, 0] - q[2, 2], q[1, 2] - q[0, 3], 0.0),
                (q[0, 2] - q[1, 3], q[1, 2] + q[0, 3], 1.0 - q[0, 0] - q[1, 1], 0.0),
                (0.0, 0.0, 0.0, 1.0),
            ),
            dtype=np.float64,
        )

    def from_translation_rotation(self, translation, rotation):
        return np.dot(self.translation_matrix(translation), self.quaternion_matrix(rotation))

    def lookup_transform(self, target_frame='base_link', source_frame='odom'):
        position, orientation = self.pb.getBasePositionAndOrientation(self.robot)
        return [position[0], position[1], position[2]], [orientation[0], orientation[1], orientation[2], orientation[3]]

    def as_matrix(self, target_frame, hdr):
        translation, rotation = self.lookup_transform(target_frame, hdr.frame_id)
        return self.from_translation_rotation(translation, rotation)

    def transform_vector3(self, target_frame, v3s: Vector3Stamped):
        mat44 = self.as_matrix(target_frame, v3s.header)
        mat44[0, 3] = 0.0
        mat44[1, 3] = 0.0
        mat44[2, 3] = 0.0
        xyz = tuple(np.dot(mat44, np.array([v3s.vector.x, v3s.vector.y, v3s.vector.z, 1.0])))[:3]
        result = Vector3Stamped()
        result.header.stamp = v3s.header.stamp
        result.header.frame_id = target_frame
        result.vector = Vector3(*xyz)
        return result

    def cmd_vel_callback(self, msg: Twist) -> None:
        self.cmd_vel_msg = msg
        self.received_cmd_vel_time = self.node.get_clock().now()

    def execute(self) -> None:
        if self.cmd_vel_msg is None or self.received_cmd_vel_time is None:
            return
        if self.node.get_clock().now() - self.received_cmd_vel_time > Duration(seconds=0.5):
            return
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
        lin_cmd_vel_in_odom = self.transform_vector3('odom', lin_vec)
        ang_cmd_vel_in_odom = self.transform_vector3('odom', ang_vec)
        self.pb.resetBaseVelocity(
            self.robot,
            [lin_cmd_vel_in_odom.vector.x, lin_cmd_vel_in_odom.vector.y, lin_cmd_vel_in_odom.vector.z],
            [ang_cmd_vel_in_odom.vector.x, ang_cmd_vel_in_odom.vector.y, ang_cmd_vel_in_odom.vector.z],
        )


# Backwards compatibility alias
cmdVelCtrl = CmdVelCtrl
