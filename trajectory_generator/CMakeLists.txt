cmake_minimum_required(VERSION 2.8.3)
project(trajectory_generator)

find_package(catkin REQUIRED
  COMPONENTS
    brics_actuator
    tf_conversions
    youbot_description
    roscpp
    ik_solver_service
    brics_actuator
    trajectory_msgs
    message_generation
)

find_package(Eigen REQUIRED)
find_package(Boost REQUIRED)


## Generate services in the 'srv' folder
add_service_files(
  FILES
  Circle.srv
  CStoCS.srv
  JStoCS.srv
  JStoJS.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
    brics_actuator
    trajectory_msgs
)

catkin_package(
  CATKIN_DEPENDS
    brics_actuator
    trajectory_msgs
    message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}  
  ../YoubotArmModel
)


add_executable(trajectory_service src/trajserv.cpp src/traj.cpp)
add_dependencies(trajectory_service ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(trajectory_service
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
  ${Boost_LIBRARIES}
)

add_executable(service_tester src/tester.cpp)
add_dependencies(service_tester ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(service_tester
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
  ${Boost_LIBRARIES}
)
