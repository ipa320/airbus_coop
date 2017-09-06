cmake_minimum_required(VERSION 2.8.3)
project(airbus_ssm_plugin)

find_package(catkin REQUIRED COMPONENTS
  airbus_ssm_core
  airbus_cobot_gui
  airbus_pyqt_extend
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
