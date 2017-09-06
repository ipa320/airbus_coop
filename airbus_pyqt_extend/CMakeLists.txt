cmake_minimum_required(VERSION 2.8.3)
project(airbus_pyqt_extend)


find_package(catkin REQUIRED COMPONENTS
  roslib
  rospy
)

catkin_python_setup()

catkin_package()

install(DIRECTORY res resources
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
