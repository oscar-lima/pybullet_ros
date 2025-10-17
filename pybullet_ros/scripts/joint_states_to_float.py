#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState


class JointStatesToFloat(Node):
    def __init__(self):
        super().__init__('joint_states_to_float')
        self.declare_parameter('input_topic', 'fake_joint_states')
        self.declare_parameter('output_suffix', '_position_controller/command')
        input_topic = self.get_parameter('input_topic').value
        self.output_suffix = self.get_parameter('output_suffix').value

        self.joint_state_msg = None
        self.previous_angles = []
        self.publishers = {}
        self.msg_received = False

        self.subscription = self.create_subscription(JointState, input_topic, self.joint_states_cb, 10)
        self.timer = self.create_timer(0.1, self.publish_joints_as_floats)

    def joint_states_cb(self, msg: JointState) -> None:
        if not self.publishers:
            for name in msg.name:
                topic_name = f'{name}{self.output_suffix}'
                self.publishers[name] = self.create_publisher(Float64, topic_name, 10)
                self.previous_angles.append(0.0)
        self.joint_state_msg = msg
        self.msg_received = True

    def publish_joints_as_floats(self) -> None:
        if not self.msg_received or self.joint_state_msg is None:
            return
        self.msg_received = False
        angles = list(self.joint_state_msg.position)
        for i, name in enumerate(self.joint_state_msg.name):
            if i >= len(self.previous_angles) or angles[i] != self.previous_angles[i]:
                publisher = self.publishers.get(name)
                if publisher is not None:
                    msg = Float64()
                    msg.data = angles[i]
                    publisher.publish(msg)
        self.previous_angles = angles


def main(args=None):
    rclpy.init(args=args)
    node = JointStatesToFloat()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt, shutting down')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
