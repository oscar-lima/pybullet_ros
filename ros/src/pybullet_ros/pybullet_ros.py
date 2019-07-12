#!/usr/bin/env python3

import sys
import math
import rospy
import time
import pybullet_data
import importlib

from std_msgs.msg import String, Float64
from std_srvs.srv import Empty
from sensor_msgs.msg import JointState

from nav_msgs.msg import Odometry

class pveControl(object):
    """helper class to receive position, velocity or effort (pve) control commands"""
    def __init__(self, joint_index, joint_name, controller_type):
        """constructor
        joint_index - stores an integer joint identifier
        joint_name - string with the name of the joint as described in urdf model
        controller_type - position, velocity or effort
        """
        assert(controller_type in ['position', 'velocity', 'effort'])
        rospy.Subscriber(joint_name + '_' + controller_type + '_controller/command',
                         Float64, self.pve_controlCB, queue_size=1)
        self.cmd = 0.0
        self.data_available = False
        self.joint_index_ = joint_index
        self.joint_name_ = joint_name

    def pve_controlCB(self, msg):
        """position, velocity or effort callback
        msg - the msg passed by the ROS network via topic publication
        """
        self.data_available = True
        self.cmd = msg.data

    def get_last_cmd(self):
        """method to fetch the last received command"""
        self.data_available = False
        return self.cmd

    def get_is_data_available(self):
        """method to retrieve flag to indicate that a command has been received"""
        return self.data_available

    def get_joint_name(self):
        """Unused method provided for completeness (pybullet works based upon joint index, not names)"""
        return self.joint_name_

    def get_joint_index(self):
        """method used to retrieve the joint int index that this class points to"""
        return self.joint_index_


class pyBulletRosWrapper(object):
    """ROS wrapper class for pybullet simulator"""
    def __init__(self):
        # import pybullet
        self.pb = importlib.import_module('pybullet')
        # setup publishers
        self.pub_joint_states = rospy.Publisher('joint_states', JointState, queue_size=1)
        self.pub_odometry = rospy.Publisher('odom', Odometry, queue_size=1)
        # get from param server the frequency at which to run the simulation
        self.loop_rate = rospy.Rate(rospy.get_param('~loop_rate', 80.0))
        # query from param server if gui is needed
        is_gui_needed = rospy.get_param('~pybullet_gui', True)
        # get from param server if user wants to pause simulation at startup
        self.pause_simulation = rospy.get_param('~pause_simulation', False)
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
        # import plugins dynamically
        self.plugins = []
        dic = rospy.get_param('~plugins', {'pybullet_ros.cmd_vel_ctrl':'cmdVelCtrl'})
        for key in dic:
            rospy.loginfo('loading %s class from %s plugin', dic[key], key)
            # create object of the imported file class
            obj = getattr(importlib.import_module(key), dic[key])(self.pb, self.robot)
            # store objects in member variable for future use
            self.plugins.append(obj)
        # get joints names and store them in dictionary
        self.joint_index_name_dictionary = self.get_joint_names()
        self.numj = len(self.joint_index_name_dictionary)
        # the max force to apply to the joint, used in velocity control
        self.force_commands = []
        max_effort_vel_mode = rospy.get_param('~max_effort_vel_mode', 50.0)
        # setup subscribers
        self.pc_subscribers = []
        self.vc_subscribers = []
        self.ec_subscribers = []
        self.joint_indices = []
        # lists to recall last received command (useful when controlling multiple joints)
        self.position_joint_commands = []
        self.velocity_joint_commands = []
        self.effort_joint_commands = []
        # joint position, velocity and effort control command individual subscribers
        for joint_index in self.joint_index_name_dictionary:
            self.position_joint_commands.append(0.0)
            self.velocity_joint_commands.append(0.0)
            self.effort_joint_commands.append(0.0)
            # used only in velocity control mode
            self.force_commands.append(max_effort_vel_mode)
            joint_name = self.joint_index_name_dictionary[joint_index]
            # create list of joints for later use in pve_ctrl_cmd(...)
            self.joint_indices.append(joint_index)
            # create position control object
            self.pc_subscribers.append(pveControl(joint_index, joint_name, 'position'))
            # create position control object
            self.vc_subscribers.append(pveControl(joint_index, joint_name, 'velocity'))
            # create position control object
            self.ec_subscribers.append(pveControl(joint_index, joint_name, 'effort'))
        # send torque = 0 to all joints at start for the robot not to fall by gravity
        self.pb.setJointMotorControlArray(bodyUniqueId=self.robot, jointIndices=self.joint_indices,
                                     controlMode=self.pb.TORQUE_CONTROL, forces=self.effort_joint_commands)
        rospy.loginfo('pybullet ROS wrapper started')

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
            rospy.logerr('mandatory param robot_urdf_path not set, will exit now')
            sys.exit()
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
        # NOTE: self collision enabled by default
        return self.pb.loadURDF(urdf_path, basePosition=[robot_pose_x, robot_pose_y, robot_pose_z], baseOrientation=robot_spawn_orientation,
                    useFixedBase=fixed_base, flags=urdf_flags)

    def get_joint_names(self):
        """filter out all non revolute joints, get their names and
        build a dictionary of joint id's to joint names.
        """
        joint_index_name_dictionary = {}
        for joint_index in range(0, self.pb.getNumJoints(self.robot)):
            info = self.pb.getJointInfo(self.robot, joint_index)
            # ensure we are dealing with a revolute joint
            if info[2] == 0: # 0 -> 'JOINT_REVOLUTE'
                # insert key, value in dictionary (joint index, joint name)
                joint_index_name_dictionary[joint_index] = info[1].decode('utf-8') # info[1] refers to joint name
        return joint_index_name_dictionary

    def pve_ctrl_cmd(self):
        """check if user has commanded a joint and forward the request to pybullet"""
        # flag to indicate there are pending position control tasks
        position_ctrl_task = False
        velocity_ctrl_task = False
        effort_ctrl_task = False
        for index, subscriber in enumerate(self.pc_subscribers):
            if subscriber.get_is_data_available():
                self.position_joint_commands[index] = subscriber.get_last_cmd()
                position_ctrl_task = True
        for index, subscriber in enumerate(self.vc_subscribers):
            if subscriber.get_is_data_available():
                self.velocity_joint_commands[index] = subscriber.get_last_cmd()
                velocity_ctrl_task = True
        for index, subscriber in enumerate(self.ec_subscribers):
            if subscriber.get_is_data_available():
                self.effort_joint_commands[index] = subscriber.get_last_cmd()
                effort_ctrl_task = True
        # forward commands to pybullet, give priority to position control cmds, then vel, at last effort
        if position_ctrl_task:
            self.pb.setJointMotorControlArray(bodyUniqueId=self.robot, jointIndices=self.joint_indices,
                                     controlMode=self.pb.POSITION_CONTROL, targetPositions=self.position_joint_commands, forces=self.force_commands)
        elif velocity_ctrl_task:
            self.pb.setJointMotorControlArray(bodyUniqueId=self.robot, jointIndices=self.joint_indices,
                                     controlMode=self.pb.VELOCITY_CONTROL, targetVelocities=self.velocity_joint_commands, forces=self.force_commands)
        elif effort_ctrl_task:
            self.pb.setJointMotorControlArray(bodyUniqueId=self.robot, jointIndices=self.joint_indices,
                                     controlMode=self.pb.TORQUE_CONTROL, forces=self.effort_joint_commands)

    def handle_reset_simulation(self, req):
        """Callback to handle the service offered by this node to reset the simulation"""
        rospy.loginfo('reseting simulation now')
        # pause simulation to prevent reading joint values with an empty world
        self.pause_simulation = True
        # remove all objects from the world and reset the world to initial conditions
        self.pb.resetSimulation()
        # load UDF model again, set gravity and floor
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

    def publish_joint_states(self):
        """query robot state and publish position, velocity and effort values to /joint_states"""
        # setup msg placeholder
        joint_msg = JointState()
        # get joint states
        for joint_index in self.joint_index_name_dictionary:
            # get joint state from pybullet
            joint_state = self.pb.getJointState(self.robot, joint_index)
            # fill msg
            joint_msg.name.append(self.joint_index_name_dictionary[joint_index])
            joint_msg.position.append(joint_state[0])# + math.pi)
            joint_msg.velocity.append(joint_state[1])
            joint_msg.effort.append(joint_state[3]) # applied effort in last sim step
        # update msg time using ROS time api
        joint_msg.header.stamp = rospy.Time.now()
        # publish joint states to ROS
        self.pub_joint_states.publish(joint_msg)

    def publishOdometry(self):
        """Query robot base position and velocity and publish odom topic and transform"""
        odom_msg = Odometry()
        odom_msg.header.frame_id = 'odom'
        odom_msg.header.stamp = rospy.Time.now()
        odom_msg.child_frame_id = 'base_link'
        # query base position from pybullet
        position, orientation = self.pb.getBasePositionAndOrientation(self.robot)
        odom_msg.pose.pose.position.x = position[0]
        odom_msg.pose.pose.position.y = position[1]
        odom_msg.pose.pose.position.z = position[2]
        odom_msg.pose.pose.orientation.x = orientation[0]
        odom_msg.pose.pose.orientation.y = orientation[1]
        odom_msg.pose.pose.orientation.z = orientation[2]
        odom_msg.pose.pose.orientation.w = orientation[3]
        # query base velocity from pybullet
        linear_vel, angular_vel = self.pb.getBaseVelocity(self.robot)
        odom_msg.twist.twist.linear.x = linear_vel[0]
        odom_msg.twist.twist.linear.y = linear_vel[1]
        odom_msg.twist.twist.linear.y = linear_vel[2]
        odom_msg.twist.twist.angular.x = angular_vel[0]
        odom_msg.twist.twist.angular.y = angular_vel[1]
        odom_msg.twist.twist.angular.z = angular_vel[2]
        self.pub_odometry.publish(odom_msg)
        # tf publication (odom to base_link)
        # TODO: tf can be broadcasted from here, but we have issues in melodic due to python 2/3 ...
        # self.br = tf.TransformBroadcaster() # need to be done from constructor
        # translation, rotation, time, child, parent
        # self.br.sendTransform(position, orientation, rospy.Time.now(), 'base_link', 'odom')

    def start_pybullet_ros_wrapper(self):
        """main simulation control cycle:
        1) check if position, velocity or effort commands are available, if so, forward to pybullet
        2) query joints state (current position, velocity and effort) and publish to ROS
        3) perform a step in pybullet simulation
        4) sleep to control the frequency of the node
        """
        while not rospy.is_shutdown():
            if not self.pause_simulation:
                # listen to position, velocity and effort control commands and forward them to pybullet
                self.pve_ctrl_cmd()
                # query joint states from pybullet and publish to ROS (/joint_states)
                self.publish_joint_states()
                # publish robot odometry
                self.publishOdometry()
                # run x plugins
                for task in self.plugins:
                    task.execute()
                # perform all the actions in a single forward dynamics simulation step such
                # as collision detection, constraint solving and integration
                self.pb.stepSimulation()
            self.loop_rate.sleep()
        # if node is killed, disconnect
        self.pb.disconnect()

def main():
    """function called by pybullet_ros_node script"""
    rospy.init_node('pybullet_ros', anonymous=False) # node name gets overrided if launched by a launch file
    pybullet_ros_interface = pyBulletRosWrapper()
    pybullet_ros_interface.start_pybullet_ros_wrapper()
