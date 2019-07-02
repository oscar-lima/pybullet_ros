# pybullet_ros

A bridge between [ROS](www.ros.org) and [PyBullet](https://github.com/bulletphysics/bullet3)

# Help needed

This project is in a early stage and presents with position, velocity and effort control interfaces.

It currently lacks integration with sensors (e.g. Lidar, RGB and RGBD cameras)

Some more work is needed and help/comments are welcome.

Main implementation is done [here](https://github.com/oscar-lima/pybullet_ros/blob/master/ros/src/pybullet_ros/pybullet_ros.py)

## Installation

This wrapper requires that you have pybullet installed, you can do so by executing:

        sudo -H pip3 install pybullet

Apart from that, just clone the repo inside your [catkin workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace),
compile (catkin build) and source your devel workspace.

## Test the simulator with position and velocity control interface

### Bringup a simple one joint example robot

This code is shipped with a simple URDF robot for testing purposes, you can run it by executing:

        roslaunch pybullet_ros bringup_robot_example.launch

You should now be able to visualise the robot in a gui similiar to the following image:

![robot](https://github.com/oscar-lima/pybullet_ros/blob/master/common/images/simple_one_joint_robot.png "one joint example robot")

### Send position control commands to the robot.

Publish a float msg to the following topic:

        rostopic pub /joint1_position_controller/command std_msgs/Float64 "data: 1.0" --once

Move in position control with convenient gui:

        rosrun pybullet_ros joint_state_publisher

A gui should pop up, use the slides to control the angle of your robot joints in position control.

NOTE: This gui should not be active if using velocity of effort commands!

### Send velocity or effort (torque) control commands to the robot.

Make sure position control interface gui publisher is not running (pybullet_ros -> joint_state_publisher)

Publish a float msg to the following topics:

velocity controller interface:

        rostopic pub /joint1_velocity_controller/command std_msgs/Float64 "data: 2.0" --once

effort controller interface:

        rostopic pub /joint1_effort_controller/command std_msgs/Float64 "data: 2000.0" --once

Done. The robot should now move in velocity or effort control mode with the desired speed/torque.

## Visualize tf data and robot model in rviz

A convenient configuration file is provided for the visualization of the single robot joint example, run it with:

        rosrun rviz rviz --display-config `rospack find pybullet_ros`/ros/config/pybullet_config.rviz

## Services offered by this node

reset simulation, of type std_srvs/Empty, which means it takes no arguments as input, it calls pybullet.resetSimulation() method.

        rosservice call /pybullet_ros/reset_simulation

pause or unpause physics, empty args, prevents the wrapper to call stepSimulation()

        rosservice call /pybullet_ros/pause_physics
        rosservice call /pybullet_ros/unpause_physics

## Topics you can use to interact with this node

```/joint_states``` (sensor_msgs/JointState) this topic is published at the ```pybullet_ros/loop_rate```
parameter frequency (see parameters section for more detail).
This topic gets listened by the robot state publisher which in turn publishes tf data to the ROS ecosystem.

## Parameters

The following parameters can be used to customize the behavior of the simulator.

~ refers to the name of the node, e.g. pybullet_ros

```~loop_rate``` - Sleep to control the frequency of how often to call pybullet.stepSimulation(), default : 10.0 (hz)

```~pybullet_gui``` - whether you want to visualize the simulation in a gui or not, default : True

```~robot_urdf_path``` - the path to load a robot at startup, default : None

```~pause_simulation``` - specify if simulation must start paused (true) or unpaused (false), default : False

```~gravity``` - the desired value of gravity for your simulation physics engine, default : -9.81

```~max_effort_vel_mode``` - the max effort (torque) to apply to the joint while in velocity control mode, default: 50.0

NOTE: max_effort_vel_mode parameter is ignored when position or effort commands are given.

# Work in progress

New features will come soon, alternatively you can contribute to this project by creating issues and pull requests, your help is welcome!

# What is missing?

- PID gains tunning interface (maybe with dynamic reconfigure?)
- Interface for sensors, e.g. RGB, RGDB camera, lidar, sonar.
- More examples with more complex robots.
- Integration of at least 1 robot with ROS moveit and navigation stack as proof of concept that integration with ROS is seamless.

# When will it be finished?

It's hard to say as at this moment I am working on this project as a hobby and only in my free time (mostly Sunday's) however
this is why the help of the community is welcome.

# How to contribute?

Check the issues and specifically for the tag "help needed", put a comment there stating that you would like to tackle this functionality
and create a pull request after finishing it.
