#!/usr/bin/env python3

"""
RGBD camera sensor simulation for pybullet_ros base on pybullet.getCameraImage()
"""

import rospy
from sensor_msgs.msg import Image

class RGBDCamera:
    def __init__(self, pybullet, robot, **kargs):
        # get "import pybullet as pb" and store in self.pb
        self.pb = pybullet
        # get robot from parent class
        self.robot = robot
        # create image msg placeholder for publication
        self.image_msg = Image()
        # get RGBD camera parameters from ROS param server
        self.image_msg.height = rospy.get_param('~rgbd_camera/resolution/width', 640)
        self.image_msg.width = rospy.get_param('~rgbd_camera/resolution/height', 480)
        # sanity checks
        assert(self.image_msg.height > 5)
        assert(self.image_msg.width > 5)
        self.image_msg.header.frame_id = rospy.get_param('~rgbd_camera/frame_id', None)
        if not self.image_msg.header.frame_id:
            rospy.logerr('Required parameter rgbd_camera/frame_id not set, will exit now...')
            rospy.signal_shutdown('Required parameter rgbd_camera/frame_id not set')
            return
        # create publisher
        self.pub_image = rospy.Publisher('rgb_image', Image, queue_size=1)
        self.image_msg.encoding = rospy.get_param('~rgbd_camera/resolution/encoding', 'rgb8')
        self.image_msg.is_bigendian = rospy.get_param('~rgbd_camera/resolution/encoding', 0)
        self.image_msg.step = rospy.get_param('~rgbd_camera/resolution/encoding', 5760)
        # variable used to run this plugin at a lower frequency, HACK
        self.count = 0

    def execute(self):
        """this function gets called from pybullet ros main update loop"""
        # run at lower frequency, camera computations are expensive
        self.count += 1
        if self.count < 100:
            return
        self.count = 0 # reset count

        # get camera image from pybullet
        pybullet_cam_resp = self.pb.getCameraImage(self.image_msg.width, self.image_msg.height)

        # fill pixel data array
        self.image_msg.data = [] # process using -> pybullet_cam_resp[n]

        # update msg time stamp
        self.image_msg.header.stamp = rospy.Time.now()
        # publish to ROS network
        self.pub_image.publish(self.image_msg)
