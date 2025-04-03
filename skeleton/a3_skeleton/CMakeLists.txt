cmake_minimum_required(VERSION 3.0.1)
project(ugv)

# This pacakge depends on a5_setup, beacuse it uses a service declared there
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  nav_msgs
  rosbag # Needed to rosbag manipulation
  roslib # Needed for ros::package::getPath
  geometry_msgs
)

## System dependencies are found with CMake's conventions
#find_package(Boost REQUIRED COMPONENTS system signals)
find_package(Eigen3 REQUIRED
            yaml-cpp REQUIRED)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated -Wdeprecated-declarations")
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
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
    #  LIBRARIES drawing_tools
#  CATKIN_DEPENDS roscpp sensor_msgs std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########


# Specify additional locations of header files
# Your package locations should be listed before other locations
include_directories(
  include
  src
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

## Declare a C++ library
add_library(${PROJECT_NAME}_laserprocessing src/laserprocessing.cpp)
add_library(${PROJECT_NAME}_controller src/controller.cpp src/ackerman.cpp)
add_library(${PROJECT_NAME}_my_logger src/my_logger.cpp)
add_library(${PROJECT_NAME}_my_publisher src/my_goals_publisher.cpp)

add_library(${PROJECT_NAME}_heuristicRRTstar src/heuristicRRTstar/heuristicRRTstar.cpp)
add_library(${PROJECT_NAME}_kinematicExpansion src/kinematicExpansion/kinematicExpansion.cpp)



## Declare a C++ executable
# Main 
add_executable(${PROJECT_NAME}_main src/main.cpp)

# Goal publisher node
###############################
add_executable(my_goals_publisher src/my_goals_publisher.cpp)# src/logger.cpp)
add_executable(my_goals_logger src/my_goals_logger.cpp)


# Add cmake target dependencies of the executable
# This enforces generating messages before building these executables
add_dependencies(${PROJECT_NAME}_main ${${PROJECT_NAME}_main_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_main ${catkin_LIBRARIES} ${PROJECT_NAME}_laserprocessing ${PROJECT_NAME}_controller ${PROJECT_NAME}_heuristicRRTstar ${PROJECT_NAME}_kinematicExpansion)
target_link_libraries(${PROJECT_NAME}_controller ${PROJECT_NAME}_laserprocessing ${PROJECT_NAME}_heuristicRRTstar ${PROJECT_NAME}_kinematicExpansion)
target_link_libraries(${PROJECT_NAME}_heuristicRRTstar yaml-cpp)
target_link_libraries(${PROJECT_NAME}_kinematicExpansion yaml-cpp)


# target_link_libraries(${PROJECT_NAME}_heuristicRRTstar ${PROJECT_NAME}_heuristicRRTstar_types)
target_link_libraries(my_goals_publisher ${catkin_LIBRARIES} ${PROJECT_NAME}_my_logger)
target_link_libraries(my_goals_logger ${catkin_LIBRARIES} ${PROJECT_NAME}_my_logger)



#############
## Testing ##
#############
## Add gtest based cpp test target and link libraries
catkin_add_gtest(${PROJECT_NAME}_test test/utest.cpp )
catkin_add_gtest(inPathTest test/inPathTest.cpp)
catkin_add_gtest(outPathTest test/outPathTest.cpp)
catkin_add_gtest(obstructedTest test/obstructedTest.cpp)
catkin_add_gtest(pathPlanningTest test/pathPlanningTest.cpp)


if(TARGET ${PROJECT_NAME}_test)

  target_link_libraries(${PROJECT_NAME}_test ${catkin_LIBRARIES} ${PROJECT_NAME}_laserprocessing )
  target_link_libraries(inPathTest ${catkin_LIBRARIES} ${PROJECT_NAME}_laserprocessing ${PROJECT_NAME}_controller)
  target_link_libraries(outPathTest ${catkin_LIBRARIES} ${PROJECT_NAME}_laserprocessing ${PROJECT_NAME}_controller)
  target_link_libraries(obstructedTest ${catkin_LIBRARIES} ${PROJECT_NAME}_laserprocessing ${PROJECT_NAME}_controller)
  target_link_libraries(pathPlanningTest ${catkin_LIBRARIES} ${PROJECT_NAME}_laserprocessing ${PROJECT_NAME}_controller ${PROJECT_NAME}_heuristicRRTstar)

  message ( STATUS " catkin_LIBRARIES ${catkin_LIBRARIES}")

endif()
