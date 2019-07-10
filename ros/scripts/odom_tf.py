#!/usr/bin/python2.7

"""
NOTE: this node can be safely deleted if tf is broadcasted from pybullet_ros.py itself
"""

import rospy
import tf

from nav_msgs.msg import Odometry

class odomToTF(object):
    '''
    subscribe to odom topic and publish tf
    '''
    def __init__(self):
        self.br = tf.TransformBroadcaster()
        rospy.Subscriber("odom", Odometry, self.odomCallback, queue_size=1)

    def odomCallback(self, odom_msg):
        translation = []
        translation.append(odom_msg.pose.pose.position.x)
        translation.append(odom_msg.pose.pose.position.y)
        translation.append(odom_msg.pose.pose.position.z)
        rotation = []
        rotation.append(odom_msg.pose.pose.orientation.x)
        rotation.append(odom_msg.pose.pose.orientation.y)
        rotation.append(odom_msg.pose.pose.orientation.z)
        rotation.append(odom_msg.pose.pose.orientation.w)
        # translation, rotation, time, child, parent
        self.br.sendTransform(translation, rotation, rospy.Time.now(), 'base_link', 'odom')

if __name__ == '__main__':
    rospy.init_node('odom_tf_broadcaster', anonymous=False)
    odom_tf_broadcaster = odomToTF()
    rospy.spin()
