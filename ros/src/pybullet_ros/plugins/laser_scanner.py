#!/usr/bin/env python3

"""
Laser scanner simulation
"""

import rospy
import math
from sensor_msgs.msg import LaserScan

class laserScanner:
    def __init__(self, pybullet, robot, **kargs):
        # get "import pybullet as pb" and store in self.pb
        self.pb = pybullet
        # get robot from parent class
        self.robot = robot
        # laser params
        laser_frameid = rospy.get_param('~laser_frameid', 'right_leg')
        self.pb_laser_link_id = self.get_link_index_from_name(laser_frameid)
        angle_min = rospy.get_param('~angle_min', -1.5707963)
        angle_max = rospy.get_param('~angle_max', 1.5707963)
        laser_angle_max = rospy.get_param('~laser_angle_max', 240) # from laser datasheet, 0 - 240 degree for hokuyo
        range_min = rospy.get_param('~range_min', 0.01)
        range_max = rospy.get_param('~range_max', 6.0)
        num_beams = rospy.get_param('~num_beams', 50) # default 512 beams for hojuyo laser
        self.beam_visualisation = rospy.get_param('~beam_visualisation', False)
        # calculate angle increment
        angle_increment = (laser_angle_max * 2.0 * math.pi)/(350.0 * num_beams)
        # register this node in the network as a publisher in /scan topic
        self.pub_laser_scanner = rospy.Publisher('scan', LaserScan, queue_size=1)
        self.laser_msg = LaserScan()
        self.laser_msg.header.frame_id = laser_frameid
        self.laser_msg.angle_min = angle_min
        self.laser_msg.angle_max = angle_max
        self.laser_msg.angle_increment = angle_increment
        self.laser_msg.time_increment = 0.01 # ?
        self.laser_msg.scan_time = 0.1 # 10 hz
        self.laser_msg.range_min = range_min
        self.laser_msg.range_max = range_max
        # self.laser_msg.intensities = []
        # fixed_joint_index_name_dic = kargs['fixed_joints']
        self.numRays = num_beams # number of beams, hokuyo has 512
        self.rayLen = range_max # max distance of the laser, Hokuyo URG-04LX-UG01 has 5.6m
        self.rayHitColor = [1, 0, 0] # red color
        self.rayMissColor = [0, 1, 0] # green color
        self.laser_position = [0.0, 0.0, 1.0] # laser coordinates
        self.angle_min = angle_min # NOTE: not outside of range: 240 degree! (but check your laser datasheet..)
        self.angle_max = angle_max
        # run this plugin at a lower frequency
        self.count = 0

    def get_link_index_from_name(self, link_name):
        # TODO: implementation missing : for now return always the first link
        return 0

    def prepare_rays(self, laser_position):
        rayFrom = []
        rayTo = []
        rayIds = []
        assert(self.angle_max > self.angle_min)
        # prepare raycast origin and end values
        for i in range(self.numRays):
            rayFrom.append(laser_position) # always same value
            t = (self.angle_max - self.angle_min) * float(i) / self.numRays
            rayTo.append([laser_position[0] + self.rayLen * math.sin(t),
                          laser_position[1] + self.rayLen * math.cos(t),
                          laser_position[2]])
            if self.beam_visualisation:
                # draw a line on pybullet gui for debug purposes
                rayIds.append(self.pb.addUserDebugLine(rayFrom[i], rayTo[i], self.rayMissColor))
        return rayIds, rayTo, rayFrom

    def execute(self):
        self.count += 1
        if self.count < 100:
            return
        self.count = 0
        """this function gets called from pybullet ros main update loop"""
        self.laser_msg.header.stamp = rospy.Time.now()
        self.laser_msg.ranges = []
        # get laser link position
        self.laser_position = self.pb.getLinkState(self.robot, self.pb_laser_link_id)[0]
        # compute start and end position of the rays
        self.pb.removeAllUserDebugItems()
        rayIds, rayTo, rayFrom = self.prepare_rays(self.laser_position)
        # raycast
        for j in range(8):
            results = self.pb.rayTestBatch(rayFrom, rayTo, j + 1)

        for i in range(self.numRays):
            hitObjectUid = results[i][0]
            if hitObjectUid < 0:
                hitPosition = [0, 0, 0]
                if self.beam_visualisation:
                    # draw a line on pybullet gui for debug purposes
                    self.pb.addUserDebugLine(rayFrom[i], rayTo[i], self.rayMissColor, replaceItemUniqueId=rayIds[i])
            else:
                hitPosition = results[i][3]
                if self.beam_visualisation:
                    # draw a line on pybullet gui for debug purposes
                    self.pb.addUserDebugLine(rayFrom[i], hitPosition, self.rayHitColor, replaceItemUniqueId=rayIds[i])
            # ranges
            # hitFraction = results[i][2]
            self.laser_msg.ranges.append(results[i][2] * self.rayLen)
        self.pub_laser_scanner.publish(self.laser_msg)
