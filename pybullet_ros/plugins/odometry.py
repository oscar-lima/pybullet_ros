#!/usr/bin/env python3

"""Query robot base pose and speed from pybullet and publish to /odom."""

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster


class SimpleOdometry:
    def __init__(self, node, pybullet, robot, **kwargs):
        self.node = node
        self.logger = node.get_logger()
        self.pb = pybullet
        self.robot = robot
        self.publisher = node.create_publisher(Odometry, 'odom', 10)
        params = kwargs.get('global_parameters', {})
        self.odom_msg = Odometry()
        self.odom_msg.header.frame_id = params.get('odom_frame', 'odom')
        self.odom_msg.child_frame_id = params.get('robot_base_frame', 'base_link')
        self.tf_broadcaster = TransformBroadcaster(node)

    def execute(self):
        now = self.node.get_clock().now().to_msg()
        self.odom_msg.header.stamp = now
        position, orientation = self.pb.getBasePositionAndOrientation(self.robot)
        self.odom_msg.pose.pose.position.x = position[0]
        self.odom_msg.pose.pose.position.y = position[1]
        self.odom_msg.pose.pose.position.z = position[2]
        self.odom_msg.pose.pose.orientation.x = orientation[0]
        self.odom_msg.pose.pose.orientation.y = orientation[1]
        self.odom_msg.pose.pose.orientation.z = orientation[2]
        self.odom_msg.pose.pose.orientation.w = orientation[3]
        linear, angular = self.pb.getBaseVelocity(self.robot)
        self.odom_msg.twist.twist.linear.x = linear[0]
        self.odom_msg.twist.twist.linear.y = linear[1]
        self.odom_msg.twist.twist.linear.z = linear[2]
        self.odom_msg.twist.twist.angular.x = angular[0]
        self.odom_msg.twist.twist.angular.y = angular[1]
        self.odom_msg.twist.twist.angular.z = angular[2]
        self.publisher.publish(self.odom_msg)

        transform = TransformStamped()
        transform.header.stamp = now
        transform.header.frame_id = self.odom_msg.header.frame_id
        transform.child_frame_id = self.odom_msg.child_frame_id
        transform.transform.translation.x = position[0]
        transform.transform.translation.y = position[1]
        transform.transform.translation.z = position[2]
        transform.transform.rotation.x = orientation[0]
        transform.transform.rotation.y = orientation[1]
        transform.transform.rotation.z = orientation[2]
        transform.transform.rotation.w = orientation[3]
        self.tf_broadcaster.sendTransform(transform)


# Backwards compatibility alias
simpleOdometry = SimpleOdometry
