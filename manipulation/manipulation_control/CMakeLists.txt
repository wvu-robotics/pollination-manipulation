cmake_minimum_required(VERSION 2.8.7)
project(manipulation_control)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
	message_generation
	message_runtime
	roscpp
	rospy
	roslib
	std_msgs
	# kinova & moveit dependencies
	cmake_modules
  geometric_shapes
  kinova_driver
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  pluginlib
  actionlib_msgs
  aruco_markers
  moveit_visual_tools
  tf2
  pcl_ros
  pcl_conversions
  manipulation_common
)

find_package(Boost REQUIRED system filesystem date_time thread)
find_package(Eigen3 REQUIRED)
find_package(kinova_driver REQUIRED)

# Generate messages in the 'msg' folder
add_message_files(
	FILES
	EECameraPose.msg
	StartPollination.msg
)

## Generate services in the 'srv' folder
# add_service_files(
# 	FILES
# )

## Generate actions in the 'action' folder
add_action_files(
  DIRECTORY action
  FILES EEGoToPose.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  actionlib_msgs
  std_msgs
	geometry_msgs
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

include_directories(
	include
	${catkin_INCLUDE_DIRS}
	${kinova_driver_INCLUDE_DIRS}
)

include_directories(SYSTEM ${Boost_INCLUDE_DIR} ${EIGEN_INCLUDE_DIRS})

link_directories(${catkin_LIBRARY_DIRS})

## Declare a C++ library
add_library(ee_go_to_pose_action src/ee_go_to_pose_action.cpp)
target_link_libraries(ee_go_to_pose_action
	${catkin_LIBRARIES}
	${Boost_LIBRARIES}
	${kinova_driver_LIBRARIES}
)

## Declare a C++ executable
add_executable(ee_go_to_pose_action_node src/ee_go_to_pose_action_node.cpp)
target_link_libraries(ee_go_to_pose_action_node
	${catkin_LIBRARIES}
	ee_go_to_pose_action
)

#############
## Install ##
#############
install(TARGETS ee_go_to_pose_action DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
