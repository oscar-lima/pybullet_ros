#!/usr/bin/env python3

import rospy
import pybullet_data
import importlib
import os

from std_srvs.srv import Empty

class pyBulletRosWrapper(object):
    """ROS wrapper class for pybullet simulator"""
    def __init__(self):
        # import pybullet
        self.pb = importlib.import_module('pybullet')
        # get from param server the frequency at which to run the simulation
        self.loop_rate = rospy.Rate(rospy.get_param('~loop_rate', 80.0))
        # query from param server if gui is needed
        is_gui_needed = rospy.get_param('~pybullet_gui', True)
        # get from param server if user wants to pause simulation at startup
        self.pause_simulation = rospy.get_param('~pause_simulation', False)
        print('\033[34m')
        # print pybullet stuff in blue
        physicsClient = self.start_gui(gui=is_gui_needed) # we dont need to store the physics client for now...
        # setup service to restart simulation
        rospy.Service('~reset_simulation', Empty, self.handle_reset_simulation)
        # setup services for pausing/unpausing simulation
        rospy.Service('~pause_physics', Empty, self.handle_pause_physics)
        rospy.Service('~unpause_physics', Empty, self.handle_unpause_physics)
        # get pybullet path in your system and store it internally for future use, e.g. to set floor
        self.pb.setAdditionalSearchPath(pybullet_data.getDataPath())
        # load robot URDF model, set gravity, and ground plane
        self.robot = self.init_pybullet_robot()
        self.connected_to_physics_server = None
        if not self.robot:
            self.connected_to_physics_server = False
            return # Error while loading urdf file
        else:
            self.connected_to_physics_server = True
        # get all revolute joint names and pybullet index
        rev_joint_index_name_dic, fixed_joint_index_name_dic = self.get_all_revolute_plus_fixed_joint_names()
        # import plugins dynamically
        self.plugins = []
        dic = rospy.get_param('~plugins', {})
        if not dic:
            rospy.logwarn('No plugins found, forgot to set param ~plugins?')
        # return to normal shell color
        print('\033[0m')
        for key in dic:
            rospy.loginfo('loading plugin: %s class from %s', dic[key], key)
            # create object of the imported file class
            obj = getattr(importlib.import_module(key), dic[key])(self.pb, self.robot,
                          rev_joints=rev_joint_index_name_dic, fixed_joints=fixed_joint_index_name_dic)
            # store objects in member variable for future use
            self.plugins.append(obj)
        rospy.loginfo('pybullet ROS wrapper initialized')

    def get_all_revolute_plus_fixed_joint_names(self):
        """filter out all non revolute joints, get their names and
        build a dictionary of joint id's to joint names.
        """
        rev_joint_index_name_dic = {}
        fixed_joint_index_name_dic = {}
        for joint_index in range(0, self.pb.getNumJoints(self.robot)):
            info = self.pb.getJointInfo(self.robot, joint_index)
            # ensure we are dealing with a revolute joint
            if info[2] == self.pb.JOINT_REVOLUTE:
                # insert key, value in dictionary (joint index, joint name)
                rev_joint_index_name_dic[joint_index] = info[1].decode('utf-8') # info[1] refers to joint name
            elif info[2] == self.pb.JOINT_FIXED:
                # insert key, value in dictionary (joint index, joint name)
                fixed_joint_index_name_dic[joint_index] = info[1].decode('utf-8') # info[1] refers to joint name
        return rev_joint_index_name_dic, fixed_joint_index_name_dic

    def handle_reset_simulation(self, req):
        """Callback to handle the service offered by this node to reset the simulation"""
        rospy.loginfo('reseting simulation now')
        self.pb.resetSimulation()
        return Empty()

    def start_gui(self, gui=True):
        """start physics engine (client) with or without gui"""
        if(gui):
            # start simulation with gui
            rospy.loginfo('Running pybullet with gui')
            rospy.loginfo('-------------------------')
            return self.pb.connect(self.pb.GUI)
        else:
            # start simulation without gui (non-graphical version)
            rospy.loginfo('Running pybullet without gui')
            # hide console output from pybullet
            rospy.loginfo('-------------------------')
            return self.pb.connect(self.pb.DIRECT)

    def init_pybullet_robot(self):
        """load robot URDF model, set gravity, and ground plane"""
        # get from param server the path to the URDF robot model to load at startup
        urdf_path = rospy.get_param('~robot_urdf_path', None)
        if urdf_path == None:
            rospy.signal_shutdown('mandatory param robot_urdf_path not set, will exit now')
        # test urdf file existance
        if not os.path.isfile(urdf_path):
            rospy.logerr('param robot_urdf_path is set, but file does not exist : ' + urdf_path)
            rospy.signal_shutdown('required robot urdf file not found')
            return None
        # get robot spawn pose from parameter server
        robot_pose_x = rospy.get_param('~robot_pose_x', 0.0)
        robot_pose_y = rospy.get_param('~robot_pose_y', 0.0)
        robot_pose_z = rospy.get_param('~robot_pose_z', 1.0)
        robot_pose_yaw = rospy.get_param('~robot_pose_yaw', 0.0)
        robot_spawn_orientation = self.pb.getQuaternionFromEuler([0.0, 0.0, robot_pose_yaw])
        fixed_base = rospy.get_param('~fixed_base', False)
        # load robot from URDF model
        # user decides if inertia is computed automatically by pybullet or custom
        if rospy.get_param('~use_intertia_from_file', False):
            # combining several boolean flags using "or" according to pybullet documentation
            urdf_flags = self.pb.URDF_USE_INERTIA_FROM_FILE | self.pb.URDF_USE_SELF_COLLISION
        else:
            urdf_flags = self.pb.URDF_USE_SELF_COLLISION
        # set gravity
        gravity = rospy.get_param('~gravity', -9.81) # get gravity from param server
        self.pb.setGravity(0, 0, gravity)
        # set floor
        self.pb.loadURDF('plane.urdf')
        # set no realtime simulation, NOTE: no need to stepSimulation if setRealTimeSimulation is set to 1
        self.pb.setRealTimeSimulation(0) # NOTE: does not currently work with effort controller, thats why is left as 0
        rospy.loginfo('\n\nloading urdf model: ' + urdf_path)
        # NOTE: self collision enabled by default
        return self.pb.loadURDF(urdf_path, basePosition=[robot_pose_x, robot_pose_y, robot_pose_z],
                                           baseOrientation=robot_spawn_orientation,
                                           useFixedBase=fixed_base, flags=urdf_flags)

    def handle_reset_simulation(self, req):
        """Callback to handle the service offered by this node to reset the simulation"""
        rospy.loginfo('reseting simulation now')
        # pause simulation to prevent reading joint values with an empty world
        self.pause_simulation = True
        # remove all objects from the world and reset the world to initial conditions
        self.pb.resetSimulation()
        # load URDF model again, set gravity and floor
        self.init_pybullet_robot()
        # resume simulation control cycle now that a new robot is in place
        self.pause_simulation = False
        return []

    def handle_pause_physics(self, req):
        """pause simulation, raise flag to prevent pybullet to execute self.pb.stepSimulation()"""
        rospy.loginfo('pausing simulation')
        self.pause_simulation = False
        return []

    def handle_unpause_physics(self, req):
        """unpause simulation, lower flag to allow pybullet to execute self.pb.stepSimulation()"""
        rospy.loginfo('unpausing simulation')
        self.pause_simulation = True
        return []

    def start_pybullet_ros_wrapper(self):
        """main simulation control cycle:
        1) check if position, velocity or effort commands are available, if so, forward to pybullet
        2) query joints state (current position, velocity and effort) and publish to ROS
        3) perform a step in pybullet simulation
        4) sleep to control the frequency of the node
        """
        while not rospy.is_shutdown():
            if not self.pause_simulation:
                # run x plugins
                for task in self.plugins:
                    task.execute()
                # perform all the actions in a single forward dynamics simulation step such
                # as collision detection, constraint solving and integration
                self.pb.stepSimulation()
            self.loop_rate.sleep()
        rospy.logwarn('killing node now...')
        # if node is killed, disconnect
        if self.connected_to_physics_server:
            self.pb.disconnect()

def main():
    """function called by pybullet_ros_node script"""
    rospy.init_node('pybullet_ros', anonymous=False) # node name gets overrided if launched by a launch file
    pybullet_ros_interface = pyBulletRosWrapper()
    pybullet_ros_interface.start_pybullet_ros_wrapper()
