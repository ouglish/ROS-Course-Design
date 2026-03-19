cmake_minimum_required(VERSION 3.0.2)
project(multi_point_nav)

find_package(catkin REQUIRED COMPONENTS
  rospy
  actionlib
  move_base_msgs
  geometry_msgs
  nav_msgs
  tf
  std_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/multi_point_nav.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
