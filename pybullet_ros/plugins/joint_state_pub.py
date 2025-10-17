#!/usr/bin/env python3

"""Query robot state and publish position, velocity and effort values to /joint_states."""

from sensor_msgs.msg import JointState


class JoinStatePublisher:
    def __init__(self, node, pybullet, robot, **kwargs):
        self.node = node
        self.logger = node.get_logger()
        self.pb = pybullet
        self.robot = robot
        self.joint_index_name_dic = kwargs['rev_joints']
        self.publisher = node.create_publisher(JointState, 'joint_states', 10)

    def execute(self) -> None:
        joint_msg = JointState()
        for joint_index in self.joint_index_name_dic:
            joint_state = self.pb.getJointState(self.robot, joint_index)
            joint_msg.name.append(self.joint_index_name_dic[joint_index])
            joint_msg.position.append(joint_state[0])
            joint_msg.velocity.append(joint_state[1])
            joint_msg.effort.append(joint_state[3])
        joint_msg.header.stamp = self.node.get_clock().now().to_msg()
        self.publisher.publish(joint_msg)


# Backwards compatibility alias
joinStatePub = JoinStatePublisher
