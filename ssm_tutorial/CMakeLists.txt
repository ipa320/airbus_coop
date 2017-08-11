cmake_minimum_required(VERSION 2.8.3)
project(ssm_tutorial)

find_package(catkin REQUIRED COMPONENTS
  rospy
  ssm_core
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS rospy ssm_core
)

install(DIRECTORY launch resources
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
