cmake_minimum_required(VERSION 2.8.12)
project(week10_quad)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  sensor_msgs
  std_msgs
  visualization_msgs
)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
message ( STATUS " CMake C++ FLAGS ${CMAKE_CXX_FLAGS}")

###################################
## catkin specific configuration ##
###################################
catkin_package()

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

#################################################################
# The library we are developing will be called controllers
add_library(controllers src/controller.cpp src/quadcopter.cpp)

# The executable will be called quiz4_a1_ex (as it substitutes PROJECT_NAME from line 4 of the CMakeLists.txt
# We only use main.cpp for the executable
add_executable(${PROJECT_NAME}_sample src/main.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_sample ${catkin_LIBRARIES} controllers)
