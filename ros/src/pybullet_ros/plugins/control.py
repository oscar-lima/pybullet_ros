#!/usr/bin/env python3

"""
position, velocity and effort control for all revolute joints on the robot
"""

import rospy
from std_msgs.msg import Float64

# NOTE: 2 classes are implemented here, scroll down to the next class (Control) to see the plugin!

class pveControl:
    """helper class to receive position, velocity or effort (pve) control commands"""
    def __init__(self, joint_index, joint_name, controller_type):
        """constructor
        joint_index - stores an integer joint identifier
        joint_name - string with the name of the joint as described in urdf model
        controller_type - position, velocity or effort
        """
        assert(controller_type in ['position', 'velocity', 'effort'])
        rospy.Subscriber(joint_name + '_' + controller_type + '_controller/command',
                         Float64, self.pve_controlCB, queue_size=1)
        self.cmd = 0.0
        self.data_available = False
        self._joint_index = joint_index
        self._joint_name = joint_name

    def pve_controlCB(self, msg):
        """position, velocity or effort callback
        msg - the msg passed by the ROS network via topic publication
        """
        self.data_available = True
        self.cmd = msg.data

    def get_last_cmd(self):
        """method to fetch the last received command"""
        self.data_available = False
        return self.cmd

    def get_is_data_available(self):
        """method to retrieve flag to indicate that a command has been received"""
        return self.data_available

    def get_joint_name(self):
        """Unused method provided for completeness (pybullet works based upon joint index, not names)"""
        return self._joint_name

    def get_joint_index(self):
        """method used to retrieve the joint int index that this class points to"""
        return self._joint_index

# plugin is implemented below
class Control:
    def __init__(self, pybullet, robot, **kargs):
        # get "import pybullet as pb" and store in self.pb
        self.pb = pybullet
        # get robot from parent class
        self.robot = robot
        # lists to recall last received command (useful when controlling multiple joints)
        self.position_joint_commands = []
        self.velocity_joint_commands = []
        self.effort_joint_commands = []
        # this parameter will be set for all robot joints
        max_effort_vel_mode = rospy.get_param('~max_effort_vel_mode', 50.0)
        # the max force to apply to the joint, used in velocity control
        self.force_commands = []
        # get joints names and store them in dictionary
        self.joint_index_name_dic = kargs['rev_joints']
        # setup subscribers
        self.pc_subscribers = []
        self.vc_subscribers = []
        self.ec_subscribers = []
        self.joint_indices = []
        # joint position, velocity and effort control command individual subscribers
        for joint_index in self.joint_index_name_dic:
            self.position_joint_commands.append(0.0)
            self.velocity_joint_commands.append(0.0)
            self.effort_joint_commands.append(0.0)
            # used only in velocity control mode
            self.force_commands.append(max_effort_vel_mode)
            joint_name = self.joint_index_name_dic[joint_index]
            # create list of joints for later use in pve_ctrl_cmd(...)
            self.joint_indices.append(joint_index)
            # create position control object
            self.pc_subscribers.append(pveControl(joint_index, joint_name, 'position'))
            # create position control object
            self.vc_subscribers.append(pveControl(joint_index, joint_name, 'velocity'))
            # create position control object
            self.ec_subscribers.append(pveControl(joint_index, joint_name, 'effort'))

    def execute(self):
        """this function gets called from pybullet ros main update loop"""
        """check if user has commanded a joint and forward the request to pybullet"""
        # flag to indicate there are pending position control tasks
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
        # forward commands to pybullet, give priority to position control cmds, then vel, at last effort
        if position_ctrl_task:
            self.pb.setJointMotorControlArray(bodyUniqueId=self.robot, jointIndices=self.joint_indices,
                                     controlMode=self.pb.POSITION_CONTROL, targetPositions=self.position_joint_commands, forces=self.force_commands)
        elif velocity_ctrl_task:
            self.pb.setJointMotorControlArray(bodyUniqueId=self.robot, jointIndices=self.joint_indices,
                                     controlMode=self.pb.VELOCITY_CONTROL, targetVelocities=self.velocity_joint_commands, forces=self.force_commands)
        elif effort_ctrl_task:
            self.pb.setJointMotorControlArray(bodyUniqueId=self.robot, jointIndices=self.joint_indices,
                                     controlMode=self.pb.TORQUE_CONTROL, forces=self.effort_joint_commands)
        
