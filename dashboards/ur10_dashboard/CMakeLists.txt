cmake_minimum_required(VERSION 2.8.3)
project(ur10_dashboard)

find_package(catkin REQUIRED COMPONENTS
  cobot_gui
  pyqt_agi_extend
  rospy
  std_msgs
)

catkin_python_setup()

catkin_package()

install(FILES dashboard_descriptor.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY resources
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
