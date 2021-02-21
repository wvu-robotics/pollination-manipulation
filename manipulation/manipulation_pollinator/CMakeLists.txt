cmake_minimum_required(VERSION 2.8.7)

project(manipulation_pollinator)

find_package(catkin REQUIRED COMPONENTS
	message_generation
	message_runtime
	roscpp
	rospy
	roslib
	std_msgs
	geometry_msgs
	manipulation_common
)

# Generate messages in the 'msg' folder
 # add_message_files(
 # FILES
 # )

## Generate services in the 'srv' folder
add_service_files(
	FILES
	PollinateFlower.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
	DEPENDENCIES
	geometry_msgs
	manipulation_common
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
	INCLUDE_DIRS include
	CATKIN_DEPENDS message_generation message_runtime roscpp rospy std_msgs geometry_msgs
)

add_definitions("-std=gnu++11")

###########
## Build ##
###########

include_directories(
	include
	src/include/manipulation_pollinator/
	${catkin_INCLUDE_DIRS}
)

add_library(maestro src/maestro.cpp)
add_dependencies(maestro ${maestro_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(maestro ${catkin_LIBRARIES}
	# ${OpenCV_LIBS}
)

add_library(pollinator_control src/pollinator_control.cpp)
add_dependencies(pollinator_control ${pollination_control_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(pollinator_control
	${catkin_LIBRARIES}
	maestro
)

## Declare a C++ executable
add_executable(end_effector_node src/end_effector.cpp)
add_dependencies(end_effector_node ${end_effector_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(end_effector_node
	${catkin_LIBRARIES}
	maestro
)

add_executable(pollinator_control_node src/pollinator_control_node.cpp)
add_dependencies(pollinator_control_node ${pollinator_control_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(pollinator_control_node
	${catkin_LIBRARIES}
	pollinator_control
)
