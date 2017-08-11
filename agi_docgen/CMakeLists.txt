cmake_minimum_required(VERSION 2.8.3)
project(agi_docgen)

find_package(catkin REQUIRED COMPONENTS
  roslib
  rospy
)

catkin_python_setup()

catkin_package()

install(DIRECTORY resources launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

catkin_install_python(PROGRAMS src/agi_docgen/docgen/index.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

