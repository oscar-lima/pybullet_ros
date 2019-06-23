#!/usr/bin/env python3

import sys
import math
import rospy
import pybullet as pb
import pybullet_data

from std_msgs.msg import String, Float64
from std_srvs.srv import Empty
from sensor_msgs.msg import JointState

class positionControl(object):
    '''
    helper class to receive position control commands
    '''
    def __init__(self, joint_index, joint_name):
        rospy.Subscriber(joint_name + '_position_controller/command', Float64, self.position_controlCB, queue_size=1)
        self.cmd = None
        self.data_available = False
        self.joint_index_ = joint_index
        self.joint_name_ = joint_name

    def position_controlCB(self, msg):
        self.data_available = True
        self.cmd = msg.data

    def get_cmd(self):
        self.data_available = False
        return self.cmd

    def is_data_available(self):
        return self.data_available

    def get_joint_name(self):
        return self.joint_name_

    def get_joint_index(self):
        return self.joint_index_


class pyBulletRosWrapper(object):
    '''
    ROS wrapper class for pybullet simulator
    '''
    def __init__(self):
        rospy.loginfo('pybullet ROS wrapper started')
        # setup publishers
        self.pub_test = rospy.Publisher('~test_publisher', String, queue_size=1)
        self.pub_joint_states = rospy.Publisher('/joint_states', JointState, queue_size=1)
        # get from param server the frequency at which to run the simulation
        self.loop_rate = rospy.Rate(rospy.get_param('~loop_rate', 10.0))
        # query from param server if gui is needed
        is_gui_needed = rospy.get_param('~pybullet_gui', True)
        # get from param server the initial URDF robot to load in environment
        urdf_path = rospy.get_param('~robot_urdf_path', None)
        if(urdf_path == None):
            rospy.logerr('mandatory param robot_urdf_path not set, will exit now')
            sys.exit()
        # get from param server if user wants to pause simulation at startup
        self.pause_simulation = rospy.get_param('~pause_simulation', False)
        # start gui
        physicsClient = self.start_gui(gui=is_gui_needed) # we dont need to store the physics client for now...
        # setup service to restart simulation
        rospy.Service('~reset_simulation', Empty, self.handle_reset_simulation)
        # setup services for pausing/unpausing simulation
        rospy.Service('~pause_physics', Empty, self.handle_pause_physics)
        rospy.Service('~unpause_physics', Empty, self.handle_unpause_physics)
        # get pybullet path in your system and store it internally for future use, e.g. to set floor
        pb.setAdditionalSearchPath(pybullet_data.getDataPath())
        # load robot from URDF model
        self.robot = pb.loadURDF(urdf_path, useFixedBase=1)
        # set realtime simulation
        pb.setRealTimeSimulation(1)
        # set gravity
        gravity = rospy.get_param('~gravity', -9.81) # get gravity from param server
        pb.setGravity(0, 0, gravity)
        # set floor
        plane = pb.loadURDF('plane.urdf')
        # do not pause simulation at startup
        self.pause_simulation = False
        # get joints names and store them in dictionary
        self.joint_index_name_dictionary = self.get_joint_names()
        self.numj = len(self.joint_index_name_dictionary)
        # remove
        rospy.loginfo('number of joints : %d', self.numj)
        rospy.loginfo('joint_index_name_dictionary : %s', self.joint_index_name_dictionary)
        # setup subscribers
        self.pc_subscribers = []
        # joint position control command individual subscribers
        for joint_index in self.joint_index_name_dictionary:
            joint_name = self.joint_index_name_dictionary[joint_index]
            # create position control object
            self.pc_subscribers.append(positionControl(joint_index, joint_name))

    def handle_reset_simulation(self, req):
        '''
        Callback to handle the service offered by this node to reset the simulation
        '''
        rospy.loginfo('reseting simulation now')
        pb.resetSimulation()
        return Empty()

    def start_gui(self, gui=True):
        '''
        start physics engine (client) with or without gui
        '''
        if(gui):
            # start simulation with gui
            rospy.loginfo('Running pybullet with gui')
            rospy.loginfo('-------------------------')
            return pb.connect(pb.GUI)
        else:
            # start simulation without gui (non-graphical version)
            rospy.loginfo('Running pybullet without gui')
            # hide console output from pybullet
            rospy.loginfo('-------------------------')
            return pb.connect(pb.DIRECT)

    def get_joint_names(self):
        '''
        filter in only joints, get their names and build a dictionary of
        joint id's -> joint names. Return the dictionary
        '''
        joint_index_name_dictionary = {}
        for joint_index in range(0, pb.getNumJoints(self.robot)):
            info = pb.getJointInfo(self.robot, joint_index)
            # ensure we are dealing with a revolute joint
            if info[2] == 0: # 0 -> 'JOINT_REVOLUTE'
                # insert key, value in dictionary (joint index, joint name)
                joint_index_name_dictionary[joint_index] = info[1].decode('utf-8') # info[1] refers to joint name
        return joint_index_name_dictionary

    def position_ctrl_cmd(self):
        '''
        '''
        joint_indices = []
        joint_commands = []
        for subscriber in self.pc_subscribers:
            if subscriber.is_data_available():
                joint_indices.append(subscriber.get_joint_index())
                joint_commands.append(subscriber.get_cmd())
        # send commands to pybullet
        pb.setJointMotorControlArray(self.robot, joint_indices, pb.POSITION_CONTROL, joint_commands)

    def handle_reset_simulation(self, req):
        '''
        Callback to handle the service offered by this node to reset the simulation
        '''
        rospy.loginfo('reseting simulation now')
        pb.resetSimulation()
        return Empty()

    def handle_pause_physics(self, req):
        '''
        pause simulation, raise flag to prevent pybullet to execute pb.stepSimulation()
        '''
        rospy.loginfo('pausing simulation')
        self.pause_simulation = False
        return Empty()

    def handle_unpause_physics(self, req):
        '''
        unpause simulation, lower flag to allow pybullet to execute pb.stepSimulation()
        '''
        rospy.loginfo('unpausing simulation')
        self.pause_simulation = True
        return Empty()

    def publish_joint_states(self):
        '''
        query robot state and publish position, velocity and effort values to /joint_states
        '''
        # setup msg placeholder
        joint_msg = JointState()
        # get joint states
        for joint_index in self.joint_index_name_dictionary:
            # get joint state from pybullet
            joint_state = pb.getJointState(self.robot, joint_index)
            # fill msg
            joint_msg.name.append(self.joint_index_name_dictionary[joint_index])
            joint_msg.position.append(joint_state[0])# + math.pi)
            joint_msg.velocity.append(joint_state[1])
            joint_msg.effort.append(joint_state[3]) # applied torque in last sim step
        # update msg time using ROS time api
        joint_msg.header.stamp = rospy.Time.now()
        # publish joint states to ROS
        self.pub_joint_states.publish(joint_msg)

    def start_pybullet_ros_wrapper(self):
        '''
        update simulation at the desired user frequency
        '''
        while not rospy.is_shutdown():
            if not self.pause_simulation:
                pb.stepSimulation()
                # query joint states from pybullet and publish to ROS (/joint_states)
                self.publish_joint_states()
                # listen to position control commands and send them to pybullet
                self.position_ctrl_cmd()
            self.loop_rate.sleep()
        # if node is killed, disconnect
        pb.disconnect()

def main():
    rospy.init_node('pybullet_ros', anonymous=False)
    pybullet_ros_interface = pyBulletRosWrapper()
    pybullet_ros_interface.start_pybullet_ros_wrapper()
