cmake_minimum_required(VERSION 3.0.2)
project(testdrive)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  erp_com
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  erp_com
)

catkin_package(
)


catkin_install_python(PROGRAMS
  src/testdrive.py
  src/testdrive2.py
  src/final_testdrive.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
