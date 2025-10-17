#!/usr/bin/env python3

"""Position, velocity and effort control for all controllable joints."""

from std_msgs.msg import Float64


class PVEControl:
    def __init__(self, node, joint_index, joint_name, controller_type):
        assert controller_type in ['position', 'velocity', 'effort']
        topic = f'{joint_name}_{controller_type}_controller/command'
        self.subscription = node.create_subscription(Float64, topic, self.pve_control_cb, 10)
        self.cmd = 0.0
        self.data_available = False
        self.joint_index = joint_index
        self.joint_name = joint_name

    def pve_control_cb(self, msg: Float64):
        self.data_available = True
        self.cmd = msg.data

    def get_last_cmd(self):
        self.data_available = False
        return self.cmd

    def get_is_data_available(self):
        return self.data_available

    def get_joint_index(self):
        return self.joint_index


class Control:
    def __init__(self, node, pybullet, robot, **kwargs):
        self.node = node
        self.logger = node.get_logger()
        self.pb = pybullet
        self.robot = robot

        global_params = kwargs.get('global_parameters', {})
        max_effort = global_params.get('max_effort', 100.0)
        if 'max_effort_vel_mode' in global_params:
            self.logger.warn('max_effort_vel_mode parameter is deprecated, please use max_effort instead')
            max_effort = global_params.get('max_effort_vel_mode', max_effort)

        self.joint_index_name_dic = {**kwargs['rev_joints'], **kwargs['prism_joints']}
        self.position_joint_commands = []
        self.velocity_joint_commands = []
        self.effort_joint_commands = []
        self.force_commands = []
        self.pc_subscribers = []
        self.vc_subscribers = []
        self.ec_subscribers = []
        self.joint_indices = []

        for joint_index, joint_name in self.joint_index_name_dic.items():
            self.position_joint_commands.append(0.0)
            self.velocity_joint_commands.append(0.0)
            self.effort_joint_commands.append(0.0)
            self.force_commands.append(max_effort)
            self.joint_indices.append(joint_index)
            self.pc_subscribers.append(PVEControl(node, joint_index, joint_name, 'position'))
            self.vc_subscribers.append(PVEControl(node, joint_index, joint_name, 'velocity'))
            self.ec_subscribers.append(PVEControl(node, joint_index, joint_name, 'effort'))

    def execute(self):
        position_ctrl_task = False
        velocity_ctrl_task = False
        effort_ctrl_task = False

        for index, subscriber in enumerate(self.pc_subscribers):
            if subscriber.get_is_data_available():
                self.position_joint_commands[index] = subscriber.get_last_cmd()
                position_ctrl_task = True
        for index, subscriber in enumerate(self.vc_subscribers):
            if subscriber.get_is_data_available():
                self.velocity_joint_commands[index] = subscriber.get_last_cmd()
                velocity_ctrl_task = True
        for index, subscriber in enumerate(self.ec_subscribers):
            if subscriber.get_is_data_available():
                self.effort_joint_commands[index] = subscriber.get_last_cmd()
                effort_ctrl_task = True

        if position_ctrl_task:
            self.pb.setJointMotorControlArray(
                bodyUniqueId=self.robot,
                jointIndices=self.joint_indices,
                controlMode=self.pb.POSITION_CONTROL,
                targetPositions=self.position_joint_commands,
                forces=self.force_commands,
            )
        elif velocity_ctrl_task:
            self.pb.setJointMotorControlArray(
                bodyUniqueId=self.robot,
                jointIndices=self.joint_indices,
                controlMode=self.pb.VELOCITY_CONTROL,
                targetVelocities=self.velocity_joint_commands,
                forces=self.force_commands,
            )
        elif effort_ctrl_task:
            self.pb.setJointMotorControlArray(
                bodyUniqueId=self.robot,
                jointIndices=self.joint_indices,
                controlMode=self.pb.TORQUE_CONTROL,
                forces=self.effort_joint_commands,
            )
