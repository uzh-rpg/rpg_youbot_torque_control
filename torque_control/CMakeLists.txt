cmake_minimum_required(VERSION 2.8.3)
project(torque_control)

find_package(catkin REQUIRED
  COMPONENTS
  	roscpp
	roslib
	genmsg	
	brics_actuator
	actionlib
	actionlib_msgs
	sensor_msgs
	trajectory_msgs
	youbot_arm_model
	std_srvs
	message_generation
	cmake_modules
)

find_package(Eigen REQUIRED)
find_package(Boost REQUIRED)

## Generate services in the 'srv' folder
add_service_files(
	FILES
	step.srv
)

## Generate actions in the 'action' folder
add_action_files(
	DIRECTORY action
	FILES torque_trajectory.action
)

generate_messages(
  DEPENDENCIES
    brics_actuator
    trajectory_msgs
    sensor_msgs
	actionlib_msgs
)

catkin_package(
  CATKIN_DEPENDS
	brics_actuator
	trajectory_msgs
	sensor_msgs
	actionlib_msgs
	youbot_arm_model
	message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_executable(torque_control src/torque_control.cpp)

add_dependencies(torque_control ${PROJECT_NAME}_generate_messages_cpp ${catkin_EXPORTED_TARGETS})

target_link_libraries(torque_control
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
  ${Boost_LIBRARIES}
)
