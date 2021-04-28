################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 2.8.3)
project(manipulation_state_machine)

add_compile_options(-std=c++11)

################################################################################
# Find catkin packages and libraries for catkin and system dependencies
################################################################################
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf
  manipulation_services
  manipulation_vision
  manipulation_mapping
  manipulation_common
  manipulation_control
  actionlib_msgs
  kinova_msgs
  genmsg
  actionlib
  tf2
  moveit_visual_tools
  message_runtime
  message_generation
  manipulation_common
)


################################################################################
# Setup for python modules and scripts
################################################################################

################################################################################
# Declare ROS messages, services and actions
################################################################################
add_service_files(
	FILES
	FindPrevisitPoses.srv
  FinalPrevisitPose.srv
)

generate_messages(
	DEPENDENCIES
  actionlib_msgs
	std_msgs
	geometry_msgs
  manipulation_common
)

################################################################################
# Declare ROS dynamic reconfigure parameters
################################################################################

################################################################################
# Declare catkin specific configuration to be passed to dependent projects
################################################################################
catkin_package(
  CATKIN_DEPENDS rospy std_msgs sensor_msgs geometry_msgs nav_msgs manipulation_services tf message_generation message_runtime
)

################################################################################
# Build
################################################################################
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


################################################################################
# Install
################################################################################
install(PROGRAMS
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


install(DIRECTORY launch/
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
