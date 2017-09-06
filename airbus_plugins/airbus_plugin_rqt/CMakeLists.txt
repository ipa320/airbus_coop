cmake_minimum_required(VERSION 2.8.3)
project(airbus_plugin_rqt)

find_package(catkin REQUIRED COMPONENTS
  airbus_cobot_gui
  rospy
  rqt_gui
  rqt_gui_py
)

catkin_python_setup()

catkin_package()

install(DIRECTORY resources
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES plugin_descriptor.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
