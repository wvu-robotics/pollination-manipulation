cmake_minimum_required(VERSION 2.8.7)
project(manipulation_vision)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
	message_generation
	message_runtime
	roscpp
	rospy
	roslib
	std_msgs
	image_transport
	cv_bridge
	sensor_msgs
	tf2_ros
	# approach_flower_action dependencies
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
	manipulation_control
	manipulation_common
	#pcl
	pcl_conversions
	pcl_ros
	rospack
)

find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED system filesystem date_time thread)
find_package(Eigen3 REQUIRED)
find_package(kinova_driver REQUIRED)

# Generate messages in the 'msg' folder
 add_message_files(
   FILES
	 Segment.msg
 )

## Generate services in the 'srv' folder
add_service_files(
	FILES
	ClassifyFlowers.srv
	SegmentFlowers.srv
	SegmentFlowersFF.srv
	ClassifyPose.srv
	# SearchForFlowers.srv
)

## Generate actions in the 'action' folder
add_action_files(
  FILES ApproachFlower.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
	DEPENDENCIES
	std_msgs
	actionlib_msgs
	manipulation_control
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
	${PYTHON_INCLUDE_DIRS}
	/usr/include/python2.7/
)

include_directories(SYSTEM ${Boost_INCLUDE_DIR} ${EIGEN_INCLUDE_DIRS})

## Declare a C++ library
add_library(segmentation src/segmentation.cpp)
target_link_libraries(segmentation ${catkin_LIBRARIES}
	${OpenCV_LIBS}
)

add_library(classification src/classification.cpp)
target_link_libraries(classification ${catkin_LIBRARIES}
	${OpenCV_LIBS}
	${PYTHON_LIBRARIES}
)

add_library(search src/search.cpp)
target_link_libraries(search ${catkin_LIBRARIES}
	${OpenCV_LIBS}
	segmentation
	classification
)

add_library(approach_flower_action src/approach_flower_action.cpp)
target_link_libraries(approach_flower_action
	${catkin_LIBRARIES}
	${Boost_LIBRARIES}
	${kinova_driver_LIBRARIES}
)

## Declare a C++ executable
add_executable(generate_lookup src/generate_lookup.cpp)
add_dependencies(generate_lookup manipulation_vision_generate_messages_cpp)
target_link_libraries(generate_lookup
	${catkin_LIBRARIES}
)

# add_executable(record_snapshot src/record_snapshot.cpp)
# add_dependencies(record_snapshot manipulation_vision_generate_messages_cpp)
# target_link_libraries(record_snapshot
# 	${catkin_LIBRARIES}
# 	${OpenCV_LIBS}
# )

add_executable(segmentation_node src/segmentation_node.cpp)
add_dependencies(segmentation_node manipulation_vision_generate_messages_cpp)
target_link_libraries(segmentation_node
	${catkin_LIBRARIES}
	segmentation
)

add_executable(classification_node src/classification_node.cpp)
add_dependencies(classification_node manipulation_vision_generate_messages_cpp)
target_link_libraries(classification_node
	${catkin_LIBRARIES}
	classification
)

add_executable(approach_flower_action_node src/approach_flower_action_node.cpp)
target_link_libraries(approach_flower_action_node
	${catkin_LIBRARIES}
	approach_flower_action
)

add_executable(test_approach_flower src/test_approach_flower.cpp)
target_link_libraries(test_approach_flower
	${catkin_LIBRARIES}
	approach_flower_action
)

add_executable(test_pose_flower_node src/test_pose_flower_node.cpp)
target_link_libraries(test_pose_flower_node
	${catkin_LIBRARIES}
)

add_executable(search_node src/search_node.cpp)
add_dependencies(search_node manipulation_vision_generate_messages_cpp)
target_link_libraries(search_node
	${catkin_LIBRARIES}
	search
)

add_executable(depthcallback_test src/depthcallback_test.cpp)
add_dependencies(depthcallback_test manipulation_vision_generate_messages_cpp)
target_link_libraries(depthcallback_test
	${catkin_LIBRARIES}
	${OpenCV_LIBS}
)
