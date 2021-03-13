#!/usr/bin/env python3

"""
plugin that is loaded one time only at the beginning
It is meant to be for you to upload your environment
"""

import rospy
import pybullet_ros.sdf.sdf_parser as sdf_parser

class Environment:
    def __init__(self, pybullet, **kargs):
        # get "import pybullet as pb" and store in self.pb
        self.pb = pybullet
        # enable soft body simulation if needed
        if rospy.get_param('~use_deformable_world', False):
            rospy.loginfo('Using deformable world (soft body simulation)')
            self.pb.resetSimulation(self.pb.RESET_USE_DEFORMABLE_WORLD)

    def load_environment(self):
        """
        set gravity, ground plane and load URDF or SDF models as required
        """
        # set gravity
        gravity = rospy.get_param('~gravity', -9.81) # get gravity from param server
        self.pb.setGravity(0, 0, gravity)
        # set floor
        self.pb.loadURDF('plane.urdf')
        # TODO: load world sdf files, (compatible gazebo .world)
        ## make sure GAZEBO_MODEL_PATH is set
        #if os.environ.get('GAZEBO_MODEL_PATH', None) == None:
            ## suggested value: /usr/share/gazebo-9/models
            #rospy.logwarn('GAZEBO_MODEL_PATH environment variable not set, models will not be able to load...')
            ## continue blue console output
            #print('\033[34m')
        #world_path = rospy.get_param('~environment', None)
        #if not world_path:
            #rospy.logwarn('environment param not set, will run pybullet with empty world')
            #return
        #if not os.path.isfile(world_path):
            #rospy.logwarn('environment file not found, world will not be loaded : ' + world_path)
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
        # give control to the user to upload custom world via code
        self.load_environment_via_code()

    def load_environment_via_code(self):
        """
        This method provides the possibility for the user to define an environment via python code
        example:
        self.pb.loadURDF(...)
        self.pb.loadSDF(...)
        self.pb.loadSoftBody(...)
        self.pb.setTimeStep(0.001)
        self.pb.setPhysicsEngineParameter(sparseSdfVoxelSize=0.25) # ? related to soft bodies
        etc...
        NOTE: is not advised to directly write code below, instead make new class and inherit from this one
              see example: environment_template.py
        """
        pass
