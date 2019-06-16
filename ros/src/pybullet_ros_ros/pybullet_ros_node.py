#!/usr/bin/env python3

import rospy
import pybullet as pb
import pybullet_data

from std_msgs.msg import String, Float64
from std_srvs.srv import Empty
from sensor_msgs.msg import JointState

class pyBulletRosWrapper(object):
    '''
    ROS wrapper class for pybullet simulator
    '''
    def __init__(self):
        rospy.loginfo('pybullet ROS wrapper started')
        # setup publishers
        self.pub_test = rospy.Publisher('~test_publisher', String, queue_size=1)
        self.pub_joint_states = rospy.Publisher('/joint_states', JointState, queue_size=1)
        # setup subscribers
        # rospy.Subscriber("~test_subscriber", String, self.emotionCallback, queue_size=1)
        rospy.Subscriber("~position_control/command", Float64, self.position_controlCB, queue_size=1)
        # get from param server the frequency at which to run the simulation
        self.loop_rate = rospy.Rate(rospy.get_param('~loop_rate', 10.0))
        # query from param server if gui is needed
        is_gui_needed = rospy.get_param('~pybullet_gui', False) # TODO: change to True by default
        # get from param server the initial URDF robot to load in environment
        #urdf_path = rospy.get_param('~robot_urdf_path', '/home/oscar/ros_ws/esterel_online_ws/src/pybullet_ros/ros/test/urdf/kr210l150.urdf')
        urdf_path = rospy.get_param('~robot_urdf_path_hack', '/home/oscar/repos_cloned/kuka_experimental/kuka_kr210_support/urdf/kr210l150.urdf')
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
        # get the number of joints in the robot
        self.numj = pb.getNumJoints(self.robot)
        # get joints names and store them in dictionary
        self.joint_index_name_dictionary = self.get_joint_names()

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
        for joint_index in range(0, self.numj):
            info = pb.getJointInfo(self.robot, joint_index)
            # ensure we are dealing with a revolute joint
            if info[2] == 0: # 0 -> 'JOINT_REVOLUTE'
                # insert key, value in dictionary (joint index, joint name)
                joint_index_name_dictionary[joint_index] = info[1].decode("utf-8") # info[1] refers to joint name
        return joint_index_name_dictionary

    def position_controlCB(self):
        '''
        TODO: this is work under progress
        callback to handle a position control command
        '''
        pass
        # pb.setJointMotorControlArray(self.robot, range(self.numj), pb.POSITION_CONTROL, targetPositions=[0.1] * self.numj)

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
        for joint_index in range(0, self.numj):
            joint_state = pb.getJointState(self.robot, joint_index)
            # if dictionary contains key, skip otherwise
            if joint_index in self.joint_index_name_dictionary:
                # fill msg
                joint_msg.name.append(self.joint_index_name_dictionary[joint_index])
                joint_msg.position.append(joint_state[0])
                joint_msg.velocity.append(joint_state[1])
                joint_msg.effort.append(joint_state[3]) # applied torque in last sim step
        # publish joint states
        self.pub_joint_states.publish(joint_msg)

    def start_pybullet_ros_wrapper(self):
        '''
        update simulation at the desired user frequency
        '''
        while not rospy.is_shutdown():
            if not self.pause_simulation:
                # step simulation
                pb.stepSimulation()
                # query and publish joint get_joint_states
                self.publish_joint_states()
            self.loop_rate.sleep()
        # if node is killed, disconnect
        pb.disconnect()

def main():
    rospy.init_node('pybullet_ros', anonymous=False)
    pybullet_ros_interface = pyBulletRosWrapper()
    pybullet_ros_interface.start_pybullet_ros_wrapper()
