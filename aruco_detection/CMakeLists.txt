cmake_minimum_required(VERSION 3.0.2)
project(aruco_detection)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  cv_bridge
  image_transport
  tf
  message_generation
)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  ArucoMarkers.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

## Catkin-specific configuration
catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs sensor_msgs geometry_msgs cv_bridge image_transport tf message_runtime
)

## Install Python scripts
catkin_install_python(PROGRAMS
  src/aruco_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

## Install config files
install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
