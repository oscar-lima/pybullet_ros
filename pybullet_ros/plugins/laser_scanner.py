#!/usr/bin/env python3

"""Laser scanner simulation based on pybullet rayTestBatch."""

import math
from typing import Dict, List

import numpy as np
from sensor_msgs.msg import LaserScan


class LaserScanner:
    def __init__(self, node, pybullet, robot, **kwargs):
        self.node = node
        self.logger = node.get_logger()
        self.pb = pybullet
        self.robot = robot

        link_ids: Dict[str, int] = kwargs['link_ids']
        laser_params = kwargs.get('parameters', {}) or kwargs.get('global_parameters', {}).get('laser', {})

        laser_frame_id = laser_params.get('frame_id')
        if not laser_frame_id:
            self.logger.error('Parameter laser.frame_id is required to initialise the laser scanner plugin')
            self.no_execution = True
            return
        if laser_frame_id not in link_ids:
            self.logger.error(f'Laser frame "{laser_frame_id}" not found in URDF model')
            self.logger.warn(f'Available frames are: {list(link_ids.keys())}')
            self.no_execution = True
            return
        self.pb_laser_link_id = link_ids[laser_frame_id]

        angle_min = float(laser_params.get('angle_min', -1.5707963))
        angle_max = float(laser_params.get('angle_max', 1.5707963))
        assert angle_max > angle_min

        self.num_rays = int(laser_params.get('num_beams', 50))
        self.range_min = float(laser_params.get('range_min', 0.03))
        self.range_max = float(laser_params.get('range_max', 5.6))
        self.beam_visualisation = bool(laser_params.get('beam_visualisation', False))

        self.publisher = node.create_publisher(LaserScan, 'scan', 10)
        self.laser_msg = LaserScan()
        self.laser_msg.header.frame_id = laser_frame_id
        self.laser_msg.angle_min = angle_min
        self.laser_msg.angle_max = angle_max
        self.laser_msg.angle_increment = (angle_max - angle_min) / self.num_rays
        self.laser_msg.range_min = self.range_min
        self.laser_msg.range_max = self.range_max
        self.laser_msg.time_increment = float(laser_params.get('time_increment', 0.01))
        self.laser_msg.scan_time = float(laser_params.get('scan_time', 0.1))

        self.ray_hit_color = [1, 0, 0]
        self.ray_miss_color = [0, 1, 0]

        self.ray_from, self.ray_to = self.prepare_rays()
        self.count = 0
        self.no_execution = False

    def prepare_rays(self):
        ray_from: List[List[float]] = []
        ray_to: List[List[float]] = []
        for n in range(0, self.num_rays):
            alpha = self.laser_msg.angle_min + n * self.laser_msg.angle_increment
            ray_from.append([self.range_min * math.cos(alpha), self.range_min * math.sin(alpha), 0.0])
            ray_to.append([self.range_max * math.cos(alpha), self.range_max * math.sin(alpha), 0.0])
        return ray_from, ray_to

    def transform_rays(self, laser_position, laser_orientation):
        laser_position = [laser_position[0], laser_position[1], laser_position[2]]
        tf_ray_from: List[List[float]] = []
        tf_ray_to: List[List[float]] = []
        rm = self.pb.getMatrixFromQuaternion(laser_orientation)
        rotation_matrix = [[rm[0], rm[1], rm[2]], [rm[3], rm[4], rm[5]], [rm[6], rm[7], rm[8]]]
        for ray in self.ray_from:
            position = np.dot(rotation_matrix, [ray[0], ray[1], ray[2]]) + laser_position
            tf_ray_from.append([position[0], position[1], position[2]])
        for ray in self.ray_to:
            position = np.dot(rotation_matrix, [ray[0], ray[1], ray[2]]) + laser_position
            tf_ray_to.append([position[0], position[1], position[2]])
        return tf_ray_from, tf_ray_to

    def execute(self) -> None:
        if getattr(self, 'no_execution', False):
            return
        self.count += 1
        if self.count < 100:
            return
        self.count = 0
        self.laser_msg.ranges = []
        if self.beam_visualisation:
            self.pb.removeAllUserDebugItems()
        laser_state = self.pb.getLinkState(self.robot, self.pb_laser_link_id)
        ray_from, ray_to = self.transform_rays(laser_state[0], laser_state[1])
        results = self.pb.rayTestBatch(ray_from, ray_to, 4)
        for i in range(self.num_rays):
            if self.beam_visualisation:
                hit_object_uid = results[i][0]
                if hit_object_uid < 0:
                    self.pb.addUserDebugLine(ray_from[i], ray_to[i], self.ray_miss_color)
                else:
                    self.pb.addUserDebugLine(ray_from[i], results[i][3], self.ray_hit_color)
            self.laser_msg.ranges.append(results[i][2] * self.laser_msg.range_max)
        self.laser_msg.header.stamp = self.node.get_clock().now().to_msg()
        self.publisher.publish(self.laser_msg)


# Backwards compatibility alias
laserScanner = LaserScanner
