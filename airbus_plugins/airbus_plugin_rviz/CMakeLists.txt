cmake_minimum_required(VERSION 2.8.3)
project(airbus_plugin_rviz)

find_package(catkin REQUIRED COMPONENTS
  airbus_cobot_gui
  roslib
  rospy
  rviz
  airbus_pyqt_extend
)

catkin_python_setup()

catkin_package()

install(FILES plugin_descriptor.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY resources
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
