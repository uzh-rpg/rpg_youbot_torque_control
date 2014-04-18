cmake_minimum_required(VERSION 2.8.3)
project(rpg_youbot_common)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  brics_actuator
)

find_package(Boost REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp brics_actuator
  DEPENDS boost
  LIBRARIES ${PROJECT_NAME}
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}  
)

# Set Sourcefiles
LIST(APPEND SOURCEFILES
  src/rpg_youbot_common.cpp
)

add_library(${PROJECT_NAME} SHARED ${SOURCEFILES})

add_dependencies(${PROJECT_NAME} SHARED ${catkin_EXPORTED_TARGETS})

target_link_libraries(
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)
