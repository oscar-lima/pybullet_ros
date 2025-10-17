#!/usr/bin/env python3

"""RGBD camera sensor simulation for pybullet_ros."""

import math
from typing import Dict

import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


class RGBDCamera:
    def __init__(self, node, pybullet, robot, **kwargs):
        self.node = node
        self.logger = node.get_logger()
        self.pb = pybullet
        self.robot = robot

        link_ids: Dict[str, int] = kwargs['link_ids']
        rgbd_config = kwargs.get('parameters', {}) or kwargs.get('global_parameters', {}).get('rgbd_camera', {})

        resolution = rgbd_config.get('resolution', {})
        self.image_msg = Image()
        self.image_msg.width = int(resolution.get('width', 640))
        self.image_msg.height = int(resolution.get('height', 480))
        if self.image_msg.width < 5 or self.image_msg.height < 5:
            raise ValueError('RGBD camera resolution must be greater than 5 pixels')

        cam_frame_id = rgbd_config.get('frame_id')
        if not cam_frame_id:
            raise RuntimeError('rgbd_camera.frame_id parameter is required')
        if cam_frame_id not in link_ids:
            raise RuntimeError(f'Camera frame "{cam_frame_id}" not found in URDF model')
        self.pb_camera_link_id = link_ids[cam_frame_id]
        self.image_msg.header.frame_id = cam_frame_id

        self.publisher = node.create_publisher(Image, 'rgb_image', 10)
        self.image_msg.encoding = rgbd_config.get('encoding', 'rgb8')
        self.image_msg.is_bigendian = int(rgbd_config.get('is_bigendian', 0))
        self.image_msg.step = int(rgbd_config.get('step', self.image_msg.width * 3))

        self.hfov = float(rgbd_config.get('hfov', 56.3))
        self.vfov = float(rgbd_config.get('vfov', 43.7))
        self.near_plane = float(rgbd_config.get('near_plane', 0.4))
        self.far_plane = float(rgbd_config.get('far_plane', 8.0))
        self.projection_matrix = self.compute_projection_matrix()

        self.image_bridge = CvBridge()
        self.count = 0

    def compute_projection_matrix(self):
        return self.pb.computeProjectionMatrix(
            left=-math.tan(math.pi * self.hfov / 360.0) * self.near_plane,
            right=math.tan(math.pi * self.hfov / 360.0) * self.near_plane,
            bottom=-math.tan(math.pi * self.vfov / 360.0) * self.near_plane,
            top=math.tan(math.pi * self.vfov / 360.0) * self.near_plane,
            nearVal=self.near_plane,
            farVal=self.far_plane,
        )

    def extract_frame(self, camera_image):
        bgr_image = np.zeros((self.image_msg.height, self.image_msg.width, 3))
        camera_image = np.reshape(camera_image[2], (camera_image[1], camera_image[0], 4))
        bgr_image[:, :, 2] = (
            (1 - camera_image[:, :, 3]) * camera_image[:, :, 2] + camera_image[:, :, 3] * camera_image[:, :, 2]
        )
        bgr_image[:, :, 1] = (
            (1 - camera_image[:, :, 3]) * camera_image[:, :, 1] + camera_image[:, :, 3] * camera_image[:, :, 1]
        )
        bgr_image[:, :, 0] = (
            (1 - camera_image[:, :, 3]) * camera_image[:, :, 0] + camera_image[:, :, 3] * camera_image[:, :, 0]
        )
        return bgr_image.astype(np.uint8)

    def compute_camera_target(self, camera_position, camera_orientation):
        target_point = [5.0, 0, 0]
        camera_position = [camera_position[0], camera_position[1], camera_position[2]]
        rm = self.pb.getMatrixFromQuaternion(camera_orientation)
        rotation_matrix = [[rm[0], rm[1], rm[2]], [rm[3], rm[4], rm[5]], [rm[6], rm[7], rm[8]]]
        return np.dot(rotation_matrix, target_point) + camera_position

    def execute(self):
        self.count += 1
        if self.count < 100:
            return
        self.count = 0
        cam_state = self.pb.getLinkState(self.robot, self.pb_camera_link_id)
        target = self.compute_camera_target(cam_state[0], cam_state[1])
        view_matrix = self.pb.computeViewMatrix(cam_state[0], target, [0, 0, 1])
        pybullet_cam_resp = self.pb.getCameraImage(
            self.image_msg.width,
            self.image_msg.height,
            view_matrix,
            self.projection_matrix,
            renderer=self.pb.ER_BULLET_HARDWARE_OPENGL,
            flags=self.pb.ER_NO_SEGMENTATION_MASK,
        )
        frame = self.extract_frame(pybullet_cam_resp)
        self.image_msg.data = self.image_bridge.cv2_to_imgmsg(frame).data
        self.image_msg.header.stamp = self.node.get_clock().now().to_msg()
        self.publisher.publish(self.image_msg)
