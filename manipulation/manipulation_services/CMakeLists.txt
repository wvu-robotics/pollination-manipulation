################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 2.8.3)
project(manipulation_services)

################################################################################
# Find catkin packages and libraries for catkin and system dependencies
################################################################################
find_package(catkin REQUIRED COMPONENTS
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs 
  message_generation
  message_runtime
  actionlib_msgs
  genmsg
)

################################################################################
# Setup for python modules and scripts
################################################################################

################################################################################
# Declare ROS messages, services and actions
################################################################################
add_service_files(
  FILES
  visual_servoing.srv
  pollinating.srv
)
add_action_files(
  FILES
  control.action
  mapping.action
  planning.action)

generate_messages(
  DEPENDENCIES
  actionlib_msgs
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs 
)

################################################################################
# Declare ROS dynamic reconfigure parameters
################################################################################

################################################################################
# Declare catkin specific configuration to be passed to dependent projects
################################################################################
catkin_package(
  CATKIN_DEPENDS std_msgs geometry_msgs nav_msgs sensor_msgs message_runtime
)

################################################################################
# Build
################################################################################

################################################################################
# Install
################################################################################

################################################################################
# Test
################################################################################
