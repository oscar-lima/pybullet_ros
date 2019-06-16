# pybullet ROS wrapper

Creates a bridge between [ROS](https://www.ros.org/) and [PyBullet](https://pybullet.org)

## Installation

This wrapper requires that you have pybullet installed, you can do so by executing:

        sudo -H pip3 install pybullet

Apart from that, just clone the repo inside your [catkin workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace), compile (catkin build) and source your devel workspace.

## Test the simulator

### Bringup R2D2 robot

This code is shipped with a simple URDF robot (r2d2) for testing purposes, you can run it by executing:

        roslaunch pybullet_ros r2d2.launch

You should now be able to visualise the robot in a gui similiar to the following image:



You can use this a sample launch file for your own application.

### Send commands to the robot.

The easiest way is to send position control commands via publishing

        rostopic pub /pybullet_ros/

## Services offered by this node

reset simulation, of type std_srvs/Empty, which means it takes no arguments as input, it calls pybullet.resetSimulation() method.

        rosservice call /pybullet_ros/reset_simulation

pause or unpause physics, empty args, prevents the wrapper to call stepSimulation()

        rosservice call /pybullet_ros/pause_physics
        rosservice call /pybullet_ros/unpause_physics

## Topics you can use to interact with this node

```/joint_states``` (sensor_msgs/JointState) this topic is published at the ```pybullet_ros/loop_rate``` parameter frequency (see parameters section for more detail). This topic gets listened by the robot state publisher which in turn publishes tf data to the ROS ecosystem.

## Parameters

The following parameters can be used to customize the behavior of the simulator.

~ refers to the name of the node, e.g. pybullet_ros

```~loop_rate``` - Sleep to control the frequency of how often to call pybullet.stepSimulation(), default : 10.0 (hz)

```~pybullet_gui``` - whether you want to visualize the simulation in a gui or not, default : True

```~robot_urdf_path``` - the path to load a robot at startup, default : None

```~pause_simulation``` - specify if simulation must start paused (true) or unpaused (false), default : False

```~gravity``` - the desired value of gravity for your simulation physics engine, default : -9.81

`````` - 
