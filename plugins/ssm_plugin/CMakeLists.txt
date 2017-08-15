cmake_minimum_required(VERSION 2.8.3)
project(ssm_plugin)

find_package(catkin REQUIRED COMPONENTS
  ssm_core
  cobot_gui
  pyqt_agi_extend
  rospy
)

catkin_python_setup()

catkin_package()

install(DIRECTORY resources
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES plugin_descriptor.xml agidoc.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

catkin_install_python(PROGRAMS scripts/ssm_interface_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
