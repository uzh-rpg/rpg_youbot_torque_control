cmake_minimum_required(VERSION 2.8.3)
project(torque_example)

find_package(catkin REQUIRED
  COMPONENTS
    brics_actuator
    tf_conversions
    youbot_description
    roscpp
    ik_solver_service
    brics_actuator
    trajectory_msgs
	rpg_youbot_common
	torque_control
	trajectory_generator
)

find_package(Eigen REQUIRED)
find_package(Boost REQUIRED)

catkin_package(
  CATKIN_DEPENDS
    brics_actuator
    trajectory_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}  
)


add_executable(gwd_traj src/ltr.cpp ../rpg_youbot_common/src/rpg_youbot_common.cpp)
target_link_libraries(gwd_traj
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
  ${Boost_LIBRARIES}
)

add_executable(circle_traj src/circle.cpp ../rpg_youbot_common/src/rpg_youbot_common.cpp)
target_link_libraries(circle_traj
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
  ${Boost_LIBRARIES}
)

