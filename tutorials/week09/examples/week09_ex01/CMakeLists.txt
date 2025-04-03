# %Tag(FULLTEXT)%
cmake_minimum_required(VERSION 2.8.3)
project(topics_tutorial_histogram)

# The below line will allow using C++11 features
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg)

## Declare a catkin package
catkin_package()

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(double_talker src/talker.cpp)
target_link_libraries(double_talker ${catkin_LIBRARIES})

add_executable(double_listener src/listener.cpp)
target_link_libraries(double_listener ${catkin_LIBRARIES})

# %EndTag(FULLTEXT)%
