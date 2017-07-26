cmake_minimum_required(VERSION 2.8.3)
project(omni_gui)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  rospy
  cobot_gui
  pyqt_agi_extend
  rviz
)

catkin_python_setup()

catkin_package()


install(PROGRAMS scripts/omni_gui_node.py
	DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY resources icons
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES plugin_descriptor.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
