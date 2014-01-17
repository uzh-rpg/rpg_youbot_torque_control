cmake_minimum_required(VERSION 2.8.3)
project(rpg_youbot_common)

find_package(catkin REQUIRED
  COMPONENTS
	tf
    roscpp
    brics_actuator
    std_msgs
)

find_package(Eigen REQUIRED)
find_package(Boost REQUIRED)

catkin_package(
  CATKIN_DEPENDS
	brics_actuator
	std_msgs
  INCLUDE_DIRS
    include/  
	
)

include_directories(
  include/
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}  
)

#add_executable(rpg_youbot_common src/rpg_youbot_common.cpp)
#target_link_libraries(torque_control
#  ${catkin_LIBRARIES}
#  ${Eigen_LIBRARIES}
#  ${Boost_LIBRARIES}
#)


