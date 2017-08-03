cmake_minimum_required(VERSION 2.8.3)
project(ur10_effector_plugin)

find_package(catkin REQUIRED COMPONENTS 
  cobot_gui
  rospy
)

catkin_python_setup()

catkin_package()

install(FILES plugin_descriptor.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY resources
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

catkin_install_python(PROGRAMS scripts/ur10_effector_plugin_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
