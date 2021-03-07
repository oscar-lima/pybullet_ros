#!/bin/bash

# delete old model
rm acrobat_robot.urdf

# generate new urdf model from xacro
rosrun xacro xacro acrobat_robot.urdf.xacro >> acrobat_robot.urdf
