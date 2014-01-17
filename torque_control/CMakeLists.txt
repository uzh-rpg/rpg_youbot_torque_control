cmake_minimum_required(VERSION 2.8.3)
project(torque_control)

find_package(catkin REQUIRED
  COMPONENTS
	youbot_description
	brics_actuator
	roscpp
	sensor_msgs
	trajectory_msgs
	actionlib
	std_srvs
	brics_actuator
	message_generation
	trajectory_msgs
	sensor_msgs
	actionlib_msgs
	roslib
)

find_package(Eigen REQUIRED)
find_package(Boost REQUIRED)

# Generate messages in the 'msg' folder
#add_message_files(
#	FILES
#	torque_trajectoryAction.msg
#	torque_trajectoryActionFeedback.msg
#	torque_trajectoryActionGoal.msg
#	torque_trajectoryActionResult.msg
#	torque_trajectoryFeedback.msg
#	torque_trajectoryGoal.msg
#	torque_trajectoryResult.msg
#)

## Generate services in the 'srv' folder
add_service_files(
	FILES
	step.srv
)

## Generate actions in the 'action' folder
add_action_files(
	FILES
	torque_trajectory.action
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
	message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}  
  ../YoubotArmModel
)

add_executable(torque_control src/torque.cpp)
add_dependencies(torque_control ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(torque_control
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
  ${Boost_LIBRARIES}
)
