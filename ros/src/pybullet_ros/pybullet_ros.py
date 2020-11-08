#!/usr/bin/env python3

import os
import importlib
import rospy
import pybullet_data
import pybullet_ros.sdf.sdf_parser as sdf_parser

from std_srvs.srv import Empty
from pybullet_ros.function_exec_manager import FuncExecManager

class pyBulletRosWrapper(object):
    """ROS wrapper class for pybullet simulator"""
    def __init__(self):
        # import pybullet
        self.pb = importlib.import_module('pybullet')
        # get from param server the frequency at which to run the simulation
        self.loop_rate = rospy.get_param('~loop_rate', 80.0)
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
        rev_joint_index_name_dic, fixed_joint_index_name_dic, link_names_to_ids_dic = self.get_properties()
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
                          rev_joints=rev_joint_index_name_dic,
                          fixed_joints=fixed_joint_index_name_dic,
                          link_ids=link_names_to_ids_dic)
            # store objects in member variable for future use
            self.plugins.append(obj)
        rospy.loginfo('pybullet ROS wrapper initialized')

    def get_properties(self):
        """
        construct 3 dictionaries:
        - joint index to joint name x2 (1 for revolute, 1 for fixed joints)
        - link name to link index dictionary
        """
        rev_joint_index_name_dic = {}
        fixed_joint_index_name_dic = {}
        link_names_to_ids_dic = {}
        for joint_index in range(0, self.pb.getNumJoints(self.robot)):
            info = self.pb.getJointInfo(self.robot, joint_index)
            # build a dictionary of link names to ids
            link_names_to_ids_dic[info[12].decode('utf-8')] = joint_index
            # ensure we are dealing with a revolute joint
            if info[2] == self.pb.JOINT_REVOLUTE:
                # insert key, value in dictionary (joint index, joint name)
                rev_joint_index_name_dic[joint_index] = info[1].decode('utf-8') # info[1] refers to joint name
            elif info[2] == self.pb.JOINT_FIXED:
                # insert key, value in dictionary (joint index, joint name)
                fixed_joint_index_name_dic[joint_index] = info[1].decode('utf-8') # info[1] refers to joint name
        return rev_joint_index_name_dic, fixed_joint_index_name_dic, link_names_to_ids_dic

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

    #def load_environment(self):
        #"""load world sdf files, (compliant with gazebo format)"""
        ## make sure GAZEBO_MODEL_PATH is set
        #if os.environ.get('GAZEBO_MODEL_PATH', None) == None:
            ## suggested value: /usr/share/gazebo-9/models
            #rospy.logwarn('GAZEBO_MODEL_PATH environment variable not set, models will not be able to load...')
            ## continue blue console output
            #print('\033[34m')
        #world_path = rospy.get_param('~environment', None)
        #if not world_path:
            #rospy.logwarn('param environment not set, will run pybullet with empty world')
            #return
        #if not os.path.isfile(world_path):
            #rospy.logwarn('file not found, world will not be loaded : ' + world_path)
            ## continue blue console output
            #print('\033[34m')
        #else:
            #rospy.loginfo('loading world : ' + world_path)
            ## get all world models
            #world = sdf_parser.SDF(file=world_path).world
            #print('succesfully imported %d models'% len(world.models))
            ## spawn sdf models in pybullet one at a time
            #for model in world.models:
                ## currently not sure how to set the required pose with loadSDF function...
                #obj_pose = [float(x) for x in model.simple_pose.split() if not x.isalpha()]
                #rospy.loginfo('loading model : ' + model.filename)
                #try:
                    #self.pb.loadSDF(model.filename)
                #except:
                    #rospy.logwarn('failed to load model : ' + model.filename + ', skipping...')

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
        # ensure urdf is not xacro, but if it is then make urdf file version out of it
        if 'xacro' in urdf_path:
            robot_description = rospy.get_param('robot_description', None)
            if not robot_description:
                rospy.logerr('required robot_description param not set')
                return None
            # remove xacro from name
            urdf_path_without_xacro = urdf_path[0:urdf_path.find('.xacro')]+urdf_path[urdf_path.find('.xacro')+len('.xacro'):]
            rospy.loginfo('generating urdf model from xacro from robot_description param server under: {0}'.format(urdf_path_without_xacro))
            try:
                urdf_file = open(urdf_path_without_xacro,'w')
            except:
                rospy.logerr('Failed to create urdf file from xacro, cannot write into destination: {0}'.format(urdf_path_without_xacro))
                return None
            urdf_file.write(robot_description)
            urdf_file.close()
            urdf_path = urdf_path_without_xacro
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
        # load environment
        #self.load_environment()
        # set no realtime simulation, NOTE: no need to stepSimulation if setRealTimeSimulation is set to 1
        self.pb.setRealTimeSimulation(0) # NOTE: does not currently work with effort controller, thats why is left as 0
        rospy.loginfo('loading urdf model: ' + urdf_path)
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

    def pause_simulation_function(self):
        return self.pause_simulation

    def start_pybullet_ros_wrapper_sequential(self):
        """
        This function is deprecated, we recommend the use of parallel plugin execution
        """
        rate = rospy.Rate(self.loop_rate)
        while not rospy.is_shutdown():
            if not self.pause_simulation:
                # run x plugins
                for task in self.plugins:
                    task.execute()
                # perform all the actions in a single forward dynamics simulation step such
                # as collision detection, constraint solving and integration
                self.pb.stepSimulation()
            rate.sleep()
        rospy.logwarn('killing node now...')
        # if node is killed, disconnect
        if self.connected_to_physics_server:
            self.pb.disconnect()

    def start_pybullet_ros_wrapper_parallel(self):
        """
        Execute plugins in parallel, however watch their execution time and warn if exceeds the deadline (loop rate)
        """
        # create object of our parallel execution manager
        exec_manager_obj = FuncExecManager(self.plugins, rospy.is_shutdown, self.pb.stepSimulation, self.pause_simulation_function,
                                           log_info=rospy.loginfo, log_warn=rospy.logwarn, log_debug=rospy.logdebug)
        # start parallel execution of all "execute" class methods in a synchronous way
        exec_manager_obj.start_synchronous_execution(loop_rate=self.loop_rate)
        # ctrl + c was pressed, exit
        rospy.logwarn('killing node now...')
        # if node is killed, disconnect
        if self.connected_to_physics_server:
            self.pb.disconnect()

    def start_pybullet_ros_wrapper(self):
        if rospy.get_param('~parallel_plugin_execution', True):
            self.start_pybullet_ros_wrapper_parallel()
        else:
            self.start_pybullet_ros_wrapper_sequential()

def main():
    """function called by pybullet_ros_node script"""
    rospy.init_node('pybullet_ros', anonymous=False) # node name gets overrided if launched by a launch file
    pybullet_ros_interface = pyBulletRosWrapper()
    pybullet_ros_interface.start_pybullet_ros_wrapper()
