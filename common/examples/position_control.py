#!/usr/bin/env python3

'''
simple 1 joint example of how to move in position control
'''

import rospkg
import pybullet as pb
import pybullet_data
import time

class testPosControl(object):
    def __init__(self):
        pb.connect(pb.GUI)
        pb.setAdditionalSearchPath(pybullet_data.getDataPath())
        urdf_path = rospkg.RosPack().get_path('pybullet_ros') + '/common/test/urdf/acrobat_robot.urdf'
        self.robot = pb.loadURDF(urdf_path, useFixedBase=1)
        pb.setRealTimeSimulation(1) # set realtime simulation
        pb.setGravity(0, 0, -9.81)
        pb.loadURDF('plane.urdf')

    def position_control_cmd(self, setpoint):
        joint_indices = [1]
        joint_commands = [setpoint]
        force_commands=[100] # maximum motor force used to reach the target value
        # position control
        #pb.setJointMotorControlArray(bodyUniqueId=self.robot, jointIndices=joint_indices,
                                     #controlMode=pb.POSITION_CONTROL, targetPositions=joint_commands, forces=force_commands)
        # velocity control
        #pb.setJointMotorControlArray(bodyUniqueId=self.robot, jointIndices=joint_indices,
                                     #controlMode=pb.VELOCITY_CONTROL, targetVelocities=joint_commands, forces=force_commands)
        # effort control
        pb.setJointMotorControlArray(bodyUniqueId=self.robot, jointIndices=joint_indices,
                                     controlMode=pb.TORQUE_CONTROL, forces=force_commands)

    def sim_for_x_secs(self, duration):
        '''
        call step sim for x amout of time
        '''
        for timestep in range(1, duration * 1000):
            pb.stepSimulation()
            time.sleep(.001) # sleep for one milisecond

if __name__ == '__main__':
    my_tester_obj = testPosControl()
    time.sleep(1)
    print('\n\nSending command now\n\n')
    my_tester_obj.position_control_cmd(100)
    #time.sleep(3)
    my_tester_obj.sim_for_x_secs(2) # simulate for 3 seconds
    pb.disconnect()
