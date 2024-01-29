cmake_minimum_required(VERSION 3.0.2)
project(imu_pub)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy sensor_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  src/imu.py
  src/imu_old.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
