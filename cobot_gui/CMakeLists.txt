cmake_minimum_required(VERSION 2.8.3)
project(cobot_gui)

find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs
  pyqt_agi_extend
  roslib
  rospy
)

catkin_python_setup()

catkin_package()

install(DIRECTORY config doc launch resources
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

catkin_install_python(PROGRAMS scripts/cobot_gui_node.py scripts/main.py src/cobot_gui/autorun.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS desktop-launch
	DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
