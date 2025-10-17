#!/usr/bin/env python3

from geometry_msgs.msg import Twist


class DiffDrive:
    def __init__(self, node, pybullet, robot, **kwargs):
        self.node = node
        self.logger = node.get_logger()
        self.pb = pybullet
        self.robot = robot

        parameters = kwargs.get('parameters', {})

        self.revolute_joints_id_name = kwargs['rev_joints']
        self.revolute_joints_name_id = {v: k for k, v in self.revolute_joints_id_name.items()}

        self.no_execution = False

        cmd_vel_topic = parameters.get('cmd_vel', 'cmd_vel')
        self.cmd_vel_subscriber = node.create_subscription(Twist, cmd_vel_topic, self.cmd_vel_callback, 10)

        self.wheel_separation = float(parameters.get('wheel_separation', 1.0))
        self.wheel_radius = float(parameters.get('wheel_radius', 1.0))

        left_joints = parameters.get('left_joints')
        right_joints = parameters.get('right_joints')
        if left_joints is None or right_joints is None:
            self.logger.error('Both left_joints and right_joints must be provided for DiffDrive controller')
            self.no_execution = True
            return
        if isinstance(left_joints, str):
            left_joints = [left_joints]
        if isinstance(right_joints, str):
            right_joints = [right_joints]

        self.joint_indices = {'left': [], 'right': []}
        for side, joint_list in (('left', left_joints), ('right', right_joints)):
            for joint_name in joint_list:
                if joint_name not in self.revolute_joints_name_id:
                    self.logger.error(f'Could not find joint {joint_name} in URDF')
                    self.no_execution = True
                    return
                self.joint_indices[side].append(self.revolute_joints_name_id[joint_name])

        self.cmd_vel = Twist()

    def cmd_vel_callback(self, msg: Twist):
        self.cmd_vel = msg

    def get_wheel_speeds(self):
        vx = self.cmd_vel.linear.x
        wz = self.cmd_vel.angular.z
        left = (vx - wz * self.wheel_separation / 2.0) / self.wheel_radius
        right = (vx + wz * self.wheel_separation / 2.0) / self.wheel_radius
        return {'left': left, 'right': right}

    def execute(self):
        if self.no_execution:
            return
        speeds = self.get_wheel_speeds()
        for side in ['left', 'right']:
            indices = self.joint_indices[side]
            self.pb.setJointMotorControlArray(
                bodyUniqueId=self.robot,
                jointIndices=indices,
                controlMode=self.pb.VELOCITY_CONTROL,
                targetVelocities=[speeds[side] for _ in range(len(indices))],
            )
