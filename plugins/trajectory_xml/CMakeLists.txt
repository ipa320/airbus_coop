cmake_minimum_required(VERSION 2.8.3)
project(trajectory_xml)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  rospy
  roslib
  cobot_gui
)
catkin_python_setup()

catkin_package()

install(FILES plugin_descriptor.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY resources save
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

catkin_install_python(PROGRAMS scripts/trajectory_xml_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
