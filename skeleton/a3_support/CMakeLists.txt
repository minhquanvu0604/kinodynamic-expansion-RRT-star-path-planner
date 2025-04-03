cmake_minimum_required(VERSION 3.0.1)
project(a3_support)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED)

## System dependencies are found with CMake's conventions
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  roscpp
  sensor_msgs
  roslib # Needed for ros::package::getPath
)


set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated -Wdeprecated-declarations")
set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")

message ( STATUS " CMake C++ FLAGS ${CMAKE_CXX_FLAGS}")


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
##  LIBRARIES msgs_and_srvs
#  CATKIN_DEPENDS grid_map_msgs nmea_msgs
##  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## C++ library
add_library(${PROJECT_NAME}_logger src/logger.cpp)

## C++ executable
add_executable(goals_publisher src/goals_publisher.cpp)# src/logger.cpp)
add_executable(goals_logger src/goals_logger.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(goals_publisher ${catkin_LIBRARIES} ${PROJECT_NAME}_logger)
target_link_libraries(goals_logger ${catkin_LIBRARIES} ${PROJECT_NAME}_logger)
