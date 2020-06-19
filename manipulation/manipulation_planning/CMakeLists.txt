cmake_minimum_required(VERSION 2.8.3)
project(manipulation_planning)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs
  cmake_modules
  geometric_shapes
  kinova_driver
  message_generation
  message_runtime
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_visual_tools
  pcl_conversions
  pcl_ros
  pluginlib
  roscpp
  roslib
  rospy
  std_msgs
  tf2
  manipulation_control
  manipulation_mapping
  manipulation_common
)

find_package(Boost REQUIRED system filesystem date_time thread)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
# )

## Generate actions in the 'action' folder
add_action_files(
  FILES
  PlanFlowerSequence.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  actionlib_msgs
  std_msgs
  geometry_msgs
  manipulation_mapping
  manipulation_common
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
	CATKIN_DEPENDS message_generation message_runtime roscpp rospy std_msgs actionlib_msgs moveit_core moveit_ros_planning_interface
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

include_directories(SYSTEM ${Boost_INCLUDE_DIR})

## Declare a C++ library
add_library(plan_flower_sequence_action src/plan_flower_sequence_action.cpp)
target_link_libraries(plan_flower_sequence_action
	${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)


## Declare a C++ executable
add_executable(plan_flower_seq_action_node src/plan_flower_seq_action_node.cpp)
target_link_libraries(plan_flower_seq_action_node
	${catkin_LIBRARIES}
	plan_flower_sequence_action
)
add_executable(test_plan_flower_sequence src/test_plan_flower_sequence.cpp)
target_link_libraries(test_plan_flower_sequence
	${catkin_LIBRARIES}
	plan_flower_sequence_action
)

#############
## Install ##
#############
install(TARGETS plan_flower_sequence_action DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
