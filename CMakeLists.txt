cmake_minimum_required(VERSION 3.8)
project(pybullet_ros)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(ament_index_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_ros REQUIRED)

ament_python_install_package(${PROJECT_NAME} PACKAGE_DIR pybullet_ros/${PROJECT_NAME})

install(DIRECTORY config launch common resource
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
