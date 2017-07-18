cmake_minimum_required(VERSION 2.8.3)
project(pyqt_agi_extend)


find_package(catkin REQUIRED COMPONENTS
  roslib
  rospy
)

catkin_python_setup()

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY res/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/res)

install(DIRECTORY resources/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/resources)
