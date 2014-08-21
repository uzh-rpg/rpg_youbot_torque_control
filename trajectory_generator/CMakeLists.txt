cmake_minimum_required(VERSION 2.8.3)
project(trajectory_generator)

find_package(catkin REQUIRED
  COMPONENTS
    brics_actuator
    tf_conversions
    roscpp
    ik_solver_service
    youbot_arm_model
    brics_actuator
    trajectory_msgs
    message_generation
    cmake_modules
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
    youbot_arm_model
    trajectory_msgs
    message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}  
)

add_executable(trajectory_service src/trajectory_server.cpp src/trajectory_generator.cpp)
add_dependencies(trajectory_service ${PROJECT_NAME}_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
target_link_libraries(trajectory_service
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
  ${Boost_LIBRARIES}
)

#add_executable(service_tester src/tester.cpp)
#add_dependencies(service_tester ${PROJECT_NAME}_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
#target_link_libraries(service_tester
#  ${catkin_LIBRARIES}
#  ${Eigen_LIBRARIES}
#  ${Boost_LIBRARIES}
#)
