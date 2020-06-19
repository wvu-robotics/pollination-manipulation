cmake_minimum_required(VERSION 2.8.3)
project(manipulation_mapping)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs
  actionlib
  aruco_markers
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
  tf
  tf2
  manipulation_control
  manipulation_common
  aruco_markers
  parameter_utils
)

find_package(Boost REQUIRED system filesystem date_time thread serialization regex timer)
find_package(Eigen3 REQUIRED)
find_package(kinova_driver REQUIRED)

find_package(GTSAM REQUIRED)
if (NOT GTSAM_FOUND)
  message(FATAL_ERROR "This program requires the GTSAM library.")
endif(NOT GTSAM_FOUND)

# add_message_files(
# 	FILES
#   EigenDecomp.msg
#   Flower.msg
# 	FlowerMap.msg
# )
#
# ## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   UpdateFlowerMap.srv
# )

## Generate actions in the 'action' folder
add_action_files(
  DIRECTORY action
  FILES BuildMap.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  actionlib_msgs
  manipulation_control
  geometry_msgs
  sensor_msgs
  manipulation_common
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
	CATKIN_DEPENDS message_generation message_runtime roscpp rospy std_msgs actionlib_msgs moveit_core moveit_ros_planning_interface
  parameter_utils
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIRS}
  ${kinova_driver_INCLUDE_DIRS}
)

include_directories(SYSTEM ${Boost_INCLUDE_DIR} ${EIGEN_INCLUDE_DIRS})

link_directories(
  ${catkin_LIBRARY_DIRS}
  ${GTSAM_LIBRARY_DIRS}
)

## Declare a C++ library


## Declare a C++ library
add_library(flower_mapper src/flower_mapper.cpp)
target_link_libraries(flower_mapper
	${catkin_LIBRARIES}
	${Boost_LIBRARIES}
	${kinova_driver_LIBRARIES}
  gtsam
)

## Declare a C++ executable
add_executable(flower_mapper_node src/flower_mapper_node.cpp)
target_link_libraries(flower_mapper_node
	${catkin_LIBRARIES}
	flower_mapper
)

#############
## Install ##
#############
install(TARGETS flower_mapper DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
