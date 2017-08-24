cmake_minimum_required(VERSION 2.8.3)
project(ssm_core)

find_package(catkin REQUIRED COMPONENTS
  rospy
  smach
  smach_ros
  actionlib
  actionlib_msgs
  std_msgs
  message_generation
)

catkin_python_setup()

add_service_files(
  FILES
  SSM_init.srv
)

add_action_files(DIRECTORY action  FILES SSM.action)

generate_messages(
  DEPENDENCIES
  std_msgs
  actionlib_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy smach smach_ros std_msgs actionlib message_runtime
)


install(DIRECTORY launch resources
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

catkin_install_python(PROGRAMS nodes/ssm_node.py nodes/ssm_action_server_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
