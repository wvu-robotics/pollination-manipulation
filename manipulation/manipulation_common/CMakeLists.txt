cmake_minimum_required(VERSION 2.8.3)
project(manipulation_common)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
	std_msgs
  roscpp
  roslib
  rospy
  message_generation
  message_runtime
  geometry_msgs
  sensor_msgs
	actionlib_msgs
  actionlib
	genmsg
)

add_action_files(DIRECTORY action FILES PlanFlowerSequence.action)



add_message_files(
	FILES
	Flower.msg
	FlowerMap.msg
  EigenDecomp.msg
  StateMachine.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
	SearchForFlowers.srv
	UpdateFlowerMap.srv
)

# ## Generate actions in the 'action' folder
# add_action_files(
#   DIRECTORY action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
	actionlib_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  # INCLUDE_DIRS include
	CATKIN_DEPENDS message_generation message_runtime roscpp rospy std_msgs geometry_msgs sensor_msgs actionlib_msgs

)

###########
## Build ##
###########

add_definitions("-std=gnu++11")

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  # include
  ${catkin_INCLUDE_DIRS}
)
