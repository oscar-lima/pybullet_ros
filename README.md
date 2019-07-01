# pybullet_ros

A bridge between [ROS](www.ros.org) and [PyBullet](https://github.com/bulletphysics/bullet3)

# Help needed

This project is in a early stage and presents with position and velocity control interface (effort interface is missing).

Some more work is needed and help/comments are welcome.

Main implementation is done [here](https://github.com/oscar-lima/pybullet_ros/blob/master/ros/src/pybullet_ros/pybullet_ros_node.py)

## Installation

This wrapper requires that you have pybullet installed, you can do so by executing:

        sudo -H pip3 install pybullet

Apart from that, just clone the repo inside your [catkin workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace),
compile (catkin build) and source your devel workspace.

## Test the simulator with position and velocity control interface

### Bringup a simple one joint example robot

This code is shipped with a simple URDF robot for testing purposes, you can run it by executing:

        roslaunch pybullet_ros simple_one_joint_robot_example.launch

You should now be able to visualise the robot in a gui similiar to the following image:

![robot](https://github.com/oscar-lima/pybullet_ros/blob/master/common/images/simple_one_joint_robot.png "one joint example robot")

### Send position control commands to the robot.

Publish a float msg to the following topic:

        rostopic pub /joint1_position_controller/command std_msgs/Float64 "data: 1.0" --once

### Send velocity control commands to the robot.

At the moment this requires a small modification in the sample launch file to prevent the joint publisher gui to open.

        rosed pybullet_ros simple_one_joint_robot_example.launch

Comment out the following line like this:

        <!--node pkg="pybullet_ros" type="joint_state_publisher" name="manual_joint_state_publisher" output="screen" required="true" /-->

Launch the file as previously explained:

        roslaunch pybullet_ros simple_one_joint_robot_example.launch

Publish a float msg to the following topic:

        rostopic pub /joint1_velocity_controller/command std_msgs/Float64 "data: 0.2" --once

Done. The robot should now move in velocity control mode with the desired speed.

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

# Work in progress

New feature will come soon, alternatively you can contribute to this project by creating issues and pull requests, your help is welcome!
