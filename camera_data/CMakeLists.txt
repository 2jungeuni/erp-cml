cmake_minimum_required(VERSION 3.0.2)
project(camera_data)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  message_filters
  cv_bridge
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy sensor_msgs message_filters cv_bridge geometry_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  src/image_analysis.py
  src/image_analysis2.py
  src/main_ROS.py
  src/turn_on_camera.py
  src/main_ROS_prs.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
