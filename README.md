# pybullet_ros

A bridge between [ROS1](https://www.ros.org/) and [PyBullet](https://pybullet.org/wordpress/)

<img src="https://github.com/oscar-lima/pybullet_ros/blob/noetic/common/images/r2d2_rviz.png" alt="drawing" width="600"/>

# Project status

This project is in a medium stage and presents with the following features:

- body velocity control - Subscription to cmd_vel topic and apply desired speed to the robot (without noise)
- joint control: Joint Position, velocity and effort control for all revolute joints on the robot
- sensors: Odometry, joint states (joint position, velocity and effort feedback), laser scanner, RGB camera image

Missing:

- sensors: Depth information (pointcloud)

Main implementation is done [here](https://github.com/oscar-lima/pybullet_ros/blob/noetic/ros/src/pybullet_ros/pybullet_ros.py)

## Installation

The following instructions have been tested under ubuntu 20.04 and [ROS noetic](http://wiki.ros.org/noetic/Installation/Ubuntu).

This wrapper requires that you have pybullet installed, you can do so by executing:

        sudo -H pip3 install pybullet

Additionally clone this repository inside your [catkin workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace),
compile (catkin build) and source your devel workspace (as you would normally do with any ROS pkg).

In case you need to simulate RGBD sensor then install opencv for python3 and ros cv bridge:

        sudo -H pip3 install opencv-python
        sudo apt install ros-noetic-cv-bridge

## Test the simulator

We provide with 2 robots for testing purposes: acrobat and r2d2, they can be found [here](https://github.com/oscar-lima/pybullet_ros/tree/noetic/common/test/urdf).

### Bringup r2d2 robot

This code is shipped with a simple URDF robot for testing purposes (r2d2), you can run it by executing:

        roslaunch pybullet_ros bringup_robot_example.launch

You should now be able to visualise the robot in a gui.

### Send position control commands to the robot.

Publish a float msg to the following topic:

        rostopic pub /head_swivel_position_controller/command std_msgs/Float64 "data: 1.0" --once

Move in position control with convenient gui:

        roslaunch pybullet_ros position_cmd_gui.launch

A gui will pop up, use the slides to control the angle of your robot joints in position control.

NOTE: This gui should not be active while sending velocity of effort commands!

### Send joint velocity or effort (torque) control commands to the robot.

NOTE: r2d2 robot has a revolute joint in the neck that can be used to test

position, velocity or effort commands as described below in the following lines:

Before sending commands, make sure position control interface gui publisher is not running!

Publish a float msg to the following topics:

velocity controller interface:

        rostopic pub /head_swivel_velocity_controller/command std_msgs/Float64 "data: 2.0" --once

effort controller interface:

        rostopic pub /head_swivel_effort_controller/command std_msgs/Float64 "data: -2000.0" --once

Done. The robot should now move in velocity or effort control mode with the desired speed/torque.

## Visualize tf data and robot model in rviz

A convenient configuration file is provided for the visualization of the example robot, run it with:

        rosrun rviz rviz --display-config `rospack find pybullet_ros`/ros/config/pybullet_config.rviz

## Topics you can use to interact with this node

```/joint_states``` (sensor_msgs/JointState) this topic is published at the ```pybullet_ros/loop_rate```
parameter frequency (see parameters section for more detail).
This topic gets listened by the robot state publisher which in turn publishes tf data to the ROS ecosystem.

```/tf``` - This wrapper broadcats all robot transformations to tf, using the robot state publisher and custom plugins.

```/scan```- Using the lidar plugin you get laser scanner readings of type sensor_msgs/LaserScan.

```/odom``` - Using the odometry plugin, robot body odometry gets published (nav_msgs/Odometry).

```/cmd_vel``` - Using the body_vel_control plugin, the robot will subscribe to cmd_vel and exert the desired velocity to the robot.

```<joint_name>_<xtype>_controller/command``` - replace "joint_name" with actual joint name and "xtype"
with [position, velocity, effort] - Using the control plugin, you can publish a joint command on this topic
and the robot will forward the instruction to the robot joint.

```/rgb_image``` - The camera image of type (sensor_msgs/Image)

## Services offered by this node

reset simulation, of type std_srvs/Empty, which means it takes no arguments as input, it calls pybullet.resetSimulation() method.

        rosservice call /pybullet_ros/reset_simulation

pause or unpause physics, empty args, prevents the wrapper to call stepSimulation()

        rosservice call /pybullet_ros/pause_physics
        rosservice call /pybullet_ros/unpause_physics

## Parameters

The following parameters can be used to customize the behavior of the simulator.

~ refers to the name of the node, e.g. pybullet_ros

```~loop_rate``` - Sleep to control the frequency of how often to call pybullet.stepSimulation(), default : 10.0 (hz)

```~pybullet_gui``` - whether you want to visualize the simulation in a gui or not, default : True

```~robot_urdf_path``` - the path to load a robot at startup, default : None

```~pause_simulation``` - specify if simulation must start paused (true) or unpaused (false), default : False

```~gravity``` - the desired value of gravity for your simulation physics engine, default : -9.81

```~max_effort_vel_mode``` - the max effort (torque) to apply to the joint while in velocity control mode, default: 50.0

```~use_intertia_from_file``` - if True pybullet will compute the inertia tensor based on mass and volume of the collision shape, default: False

```~robot_pose_x``` - The position where to spawn the robot in the world in m, default: 0.0

```~robot_pose_y``` - The position where to spawn the robot in the world in m, default: 0.0

```~robot_pose_z``` - The position where to spawn the robot in the world in m, default: 1.0

```~robot_pose_yaw``` - The orientation where to spawn the robot in the world, default: 0.0

```~fixed_base``` - If true, the first link of the robot will be fixed to the center of the world, useful for non movable robots default: False

NOTE: max_effort_vel_mode parameter is ignored when position or effort commands are given.

Experimental (does not work at the moment) :

```~environment``` - (work in progress) Load a world file, which is a collection of sdf objects, default: Empty world

# pybullet ros plugins

What is a pybullet ros plugin?

At the core of pybullet ros, we have the following workflow:

<img src="https://github.com/oscar-lima/pybullet_ros/blob/noetic/common/images/main_loop.png" alt="drawing" width="200"/>

Basically we iterate over all registered plugins, run their execute function, and after doing it for all plugins we step the simulation one time step.

# Plugin creation

This section shows you how you can create your own plugin, to extend this library with your own needs.

NOTE: Before creating a pybullet_ros plugin, make sure your plugin does not exist already
[check available pybullet_ros plugins here](https://github.com/oscar-lima/pybullet_ros/tree/noetic/ros/src/pybullet_ros/plugins).

To ease the process, we provide with a template [here](https://github.com/oscar-lima/pybullet_ros/blob/noetic/ros/src/pybullet_ros/plugins/plugin_template.py).

Copy the template and follow this instructions:

1. roscd pybullet_ros/ros/src/pybullet_ros/plugins && cp plugin_template.py my_awesome_plugin.py

2. add it  to param server

    roscd pybullet_ros/ros/config && gedit pybullet_params_example.yaml

Extend "plugins" param to add yours, e.g:

    plugins: {  pybullet_ros.plugins.body_vel_control: cmdVelCtrl,
                pybullet_ros.plugins.odometry: simpleOdometry,
                pybullet_ros.plugins.control: Control}
    
    plugins: {  pybullet_ros.plugins.body_vel_control: cmdVelCtrl,
                pybullet_ros.plugins.odometry: simpleOdometry,
                pybullet_ros.plugins.control: Control,
                pybullet_ros.plugins.plugin_template: pluginTemplate}

3. Thats all! you receive a pointer to the robot and to the import of pybullet itself, e.g.

        import pybullet as pb -> self.pb
        self.robot -> self.pb.loadURDF(urdf_path, basePosition=[0,0,0], baseOrientation=[0,0,0,1], ...)

Using the pybullet [documentation](https://docs.google.com/document/d/10sXEhzFRSnvFcl3XxNGhnD4N2SedqwdAvK3dsihxVUA/edit#) you should be able
to access all the functionality that the pybullet api provides.

## NOTE about the multiple r2d2 urdf models in the web

As you might have already noticed, there are multiple r2d2 urdf models in the web, for instance the one that
ROS [uses](http://wiki.ros.org/urdf/Tutorials/Building%20a%20Visual%20Robot%20Model%20with%20URDF%20from%20Scratch) to
teach new users about URDF, however is missing collision and inertia tags. Another one can be found under pybullet repo
[data folder](https://github.com/bulletphysics/bullet3/blob/master/data/r2d2.urdf) but that model does not follow
[ROS conventions](https://www.ros.org/reps/rep-0103.html#axis-orientation), in particular "x" axis moves the robot forward and "y" axis moves it to the left.
We have created our own r2d2 and included a lidar on its base to be able to test the laser scanner plugin.

## Wait a second... bullet is already integrated in gazebo. Why do I need this repository at all?

Well thats true, bullet is integrated in gazebo, they have much more plugins available and their api runs much faster as it is implemented in C++.

I myself also use gazebo on a daily basis! , however probably some reasons why this repository be useful are because is very easy and fast to configure a rapid prototype.

Your urdf model does not need to be extended with gazebo tags, installation is extremely easy from pypi and there are lots of examples in pybullet available (see [here](https://github.com/bulletphysics/bullet3/tree/master/examples/pybullet/examples)). Additionally, its in python! so is easier and faster to develop + the pybullet documentation is better.
