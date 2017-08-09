cmake_minimum_required(VERSION 2.8.3)
project(ssm_core)

find_package(catkin REQUIRED COMPONENTS
  rospy
  smach
  smach_ros
  message_generation
)

catkin_python_setup()

add_service_files(
  FILES
  SSM_init.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs   
)

catkin_package(
  CATKIN_DEPENDS rospy smach smach_ros message_runtime std_msgs
)

install(DIRECTORY launch resources
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

catkin_install_python(PROGRAMS src/ssm_core/ssm_main.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)