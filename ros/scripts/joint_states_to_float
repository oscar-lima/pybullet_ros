#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState

class JointStatesToFloat:
    def __init__(self):
        self.rate = rospy.Rate(10)
        self.msg_received = False
        self.joint_state_msg = None
        self.previous_angles = []
        rospy.Subscriber('fake_joint_states', JointState, self.JointStatesCB)
        rospy.loginfo('waiting for first joint state msg to be published')
        self.wait_for_first_msg()
        rospy.loginfo('joint state msg received, proceeding')
        # init previous angle list
        for i in range(len(self.joint_state_msg.position)):
            self.previous_angles.append(0.0)
        # create one publisher per joint name
        self.pub_list = []
        for name in self.joint_state_msg.name:
            self.pub_list.append(rospy.Publisher(name + '_position_controller/command', Float64, queue_size=1))

    def wait_for_first_msg(self):
        while not self.msg_received:
            # wait until at least one joint state msg is received
            self.rate.sleep()

    def JointStatesCB(self, msg):
        self.joint_state_msg = msg
        self.msg_received = True

    def publishJointsAsFloats(self):
        # extract desired angles from joint state msg
        angles = []
        for angle in self.joint_state_msg.position:
            angles.append(angle)
        # compare previous angles to see if there was any change
        changes = []
        for i, previous_angle in enumerate(self.previous_angles):
            changes.append(previous_angle != angles[i])
        # publish desired angles to float topics
        for i, pub in enumerate(self.pub_list):
            if changes[i]: # compute only if angle changed
                float_msg = Float64()
                float_msg.data = angles[i]
                pub.publish(float_msg)
        # to keep track if the angle changed or not
        self.previous_angles = angles

    def start(self):
        while not rospy.is_shutdown():
            if self.msg_received:
                # lower flag
                self.msg_received = False
                self.publishJointsAsFloats()
            self.rate.sleep()

if __name__ == '__main__':
    rospy.init_node('joint_states_to_float', anonymous=False)
    jstf = JointStatesToFloat()
    jstf.start()
