#!/usr/bin/env python3

"""
Laser scanner simulation
"""

import rospy
import math
import numpy as np
from sensor_msgs.msg import LaserScan

class laserScanner:
    def __init__(self, pybullet, robot, **kargs):
        # get "import pybullet as pb" and store in self.pb
        self.pb = pybullet
        # get robot from parent class
        self.robot = robot
        # laser params
        laser_frameid = rospy.get_param('~laser_frameid', None) # laser reference frame, has to be an existing link
        if not laser_frameid:
            rospy.logerr('required parameter laser_frameid not set, will exit now')
            rospy.signal_shutdown('required param laser_frameid not set')
            return
        # get pybullet laser link id from its name
        self.pb_laser_link_id = self.get_link_index_from_name(laser_frameid)
        # laser field of view (fov)
        angle_min = rospy.get_param('~angle_min', -1.5707963) # limit laser to a shorter range
        angle_max = rospy.get_param('~angle_max', 1.5707963)
        assert(angle_max > angle_min)
        laser_angle_max = rospy.get_param('~laser_angle_max', 240) # from laser datasheet, 0 - 240 degree for hokuyo
        range_max = rospy.get_param('~range_max', 6.0)
        num_beams = rospy.get_param('~num_beams', 50) # default 512 beams for hokuyo laser
        self.beam_visualisation = rospy.get_param('~beam_visualisation', False)
        # register this node in the network as a publisher in /scan topic
        self.pub_laser_scanner = rospy.Publisher('scan', LaserScan, queue_size=1)
        self.laser_msg = LaserScan()
        self.laser_msg.header.frame_id = laser_frameid
        self.laser_msg.angle_min = angle_min
        self.laser_msg.angle_max = angle_max
        self.laser_msg.angle_increment = (angle_max - angle_min) / num_beams
        self.laser_msg.time_increment = 0.01 # ?
        self.laser_msg.scan_time = 0.1 # 10 hz
        self.laser_msg.range_min = rospy.get_param('~range_min', 0.01)
        self.laser_msg.range_max = range_max
        # self.laser_msg.intensities = []
        # fixed_joint_index_name_dic = kargs['fixed_joints']
        self.numRays = num_beams # number of beams, hokuyo has 512
        self.rayLen = range_max # max distance of the laser, Hokuyo URG-04LX-UG01 has 5.6m
        self.rayHitColor = [1, 0, 0] # red color
        self.rayMissColor = [0, 1, 0] # green color
        self.laser_position = [0.0, 0.0, 1.0] # laser coordinates
        # compute rays end beam position
        self.rayFrom, self.rayTo = self.prepare_rays()
        # variable used to run this plugin at a lower frequency
        self.count = 0

    def get_link_index_from_name(self, link_name):
        # TODO: implementation missing, hand tuned to return the box in the head of R2D2
        #return 15
        return 0

    def prepare_rays(self):
        """assume laser is in the origin and compute its x, y beam end position"""
        # prepare raycast origin and end values
        rayFrom = []
        rayTo = []
        for n in range(0, self.numRays):
            rayFrom.append([0, 0, 0])
            alpha = self.laser_msg.angle_min + n * self.laser_msg.angle_increment
            rayTo.append([self.laser_msg.range_max * math.cos(alpha),
                          self.laser_msg.range_max * math.sin(alpha), 0.0])
        return rayFrom, rayTo

    def transform_rays(self, laser_position, laser_orientation):
        """transform rays from reference frame using pybullet functions (not tf)"""
        # add a bit of distance on z to avoid collision with model, HACK
        laser_position = [laser_position[0], laser_position[1], laser_position[2] + 0.1]
        TFrayFrom = []
        TFrayTo = []
        rm = self.pb.getMatrixFromQuaternion(laser_orientation)
        rotation_matrix = [[rm[0], rm[1], rm[2]],[rm[3], rm[4], rm[5]],[rm[6], rm[7], rm[8]]]
        for ray in self.rayTo:
            TFrayFrom.append(laser_position)
            position = np.dot(rotation_matrix, [ray[0], ray[1], ray[2]]) + laser_position
            TFrayTo.append([position[0], position[1], position[2]])
        return TFrayFrom, TFrayTo

    def execute(self):
        """this function gets called from pybullet ros main update loop"""
        # run at lower frequency, laser computations are expensive
        self.count += 1
        if self.count < 100:
            return
        self.count = 0 # reset count
        # remove any previous laser data if any
        self.laser_msg.ranges = []
        # remove previous beam lines from screen
        if self.beam_visualisation:
            self.pb.removeAllUserDebugItems()
        # get laser link position
        laser_state = self.pb.getLinkState(self.robot, self.pb_laser_link_id)
        # transform start and end position of the rays which were generated considering laser at the origin
        rayFrom, rayTo  = self.transform_rays(laser_state[0], laser_state[1]) # position + orientation
        # raycast using 4 threads
        results = self.pb.rayTestBatch(rayFrom, rayTo, 4)
        for i in range(self.numRays):
            if self.beam_visualisation:
                hitObjectUid = results[i][0]
                if hitObjectUid < 0:
                    # draw a line on pybullet gui for debug purposes in green because it did not hit any obstacle
                    self.pb.addUserDebugLine(rayFrom[i], rayTo[i], self.rayMissColor)
                else:
                    # draw a line on pybullet gui for debug purposes in red because it hited obstacle, results[i][3] -> hitPosition
                    self.pb.addUserDebugLine(rayFrom[i], results[i][3], self.rayHitColor)
            # compute laser ranges from hitFraction -> results[i][2]
            self.laser_msg.ranges.append(results[i][2] * self.rayLen)
        # update laser time stamp with current time
        self.laser_msg.header.stamp = rospy.Time.now()
        # publish scan
        self.pub_laser_scanner.publish(self.laser_msg)
