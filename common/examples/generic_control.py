#!/usr/bin/env python3

'''

simple 1 joint example of how to move in position, velocity and effort control
This file is not directly usable, try the companion files located in the same folder:

- python3 position_control.py
- python3 velocity_control.py
- python3 effort_control.py

'''

import rospkg # only used to get URDF path, this file is supposed to be ROS independent
import pybullet as pb
import pybullet_data
import time

class pveControl(object):
    def __init__(self):
        pb.connect(pb.GUI)
        pb.setAdditionalSearchPath(pybullet_data.getDataPath())
        urdf_path = rospkg.RosPack().get_path('pybullet_ros') + '/common/test/urdf/acrobat_robot.urdf'
        self.robot = pb.loadURDF(urdf_path, useFixedBase=1)
        #pb.setRealTimeSimulation(1) # set realtime simulation, NOTE: does not currently work with effort controller
        pb.setGravity(0, 0, -9.81)
        pb.loadURDF('plane.urdf')

    def test_position_control(self):
        # position control
        pb.setJointMotorControlArray(bodyUniqueId=self.robot, jointIndices=[1],
                                     controlMode=pb.POSITION_CONTROL, targetPositions=[1.57], forces=[100])

    def test_velocity_control(self):
        # velocity control
        pb.setJointMotorControlArray(bodyUniqueId=self.robot, jointIndices=[1],
                                     controlMode=pb.VELOCITY_CONTROL, targetVelocities=[2.0], forces=[100])

    def test_effort_control(self):
        # effort control
        pb.setJointMotorControlArray(bodyUniqueId=self.robot, jointIndices=[1],
                                     controlMode=pb.TORQUE_CONTROL, forces=[4000])

    def sim_for_x_secs(self, duration):
        '''
        call step sim for x amout of time
        '''
        for timestep in range(1, duration * 100):
            pb.stepSimulation()
            time.sleep(0.01) # sleep for ten miliseconds

    def disconnect(self):
        '''
        method used to expose the pybullet disconnect function
        '''
        pb.disconnect()

