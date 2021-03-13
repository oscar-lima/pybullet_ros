# test

## Instructions on how to run this example:

Launch prismatic joint example:

        roslaunch pybullet_ros bringup_robot_example.launch config_file:=`rospack find pybullet_ros`/common/test/urdf/prismatic_joint/prismatic_params.yaml robot_urdf_path:=`rospack find pybullet_ros`/common/test/urdf/prismatic_joint/prismatic.urdf robot_pose_z:=0.05

Actuate prismatic joint with gui:

        roslaunch pybullet_ros position_cmd_gui.launch
