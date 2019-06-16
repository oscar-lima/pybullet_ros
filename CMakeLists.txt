cmake_minimum_required(VERSION 2.8.3)
project(pybullet_ros)

find_package(catkin REQUIRED COMPONENTS
    std_msgs
    message_generation
    genmsg
)

catkin_python_setup()

add_service_files(
    DIRECTORY ros/srv
    FILES ResetSimulation.srv
)

generate_messages(
   DEPENDENCIES
)

catkin_package(
    CATKIN_DEPENDS message_generation message_runtime std_msgs genmsg
)
