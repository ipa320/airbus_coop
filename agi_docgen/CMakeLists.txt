cmake_minimum_required(VERSION 2.8.3)
project(agi_docgen)

find_package(catkin REQUIRED COMPONENTS
  roslib
  rospy
)

catkin_python_setup()

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############

install(DIRECTORY resources
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

