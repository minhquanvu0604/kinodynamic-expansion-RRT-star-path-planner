cmake_minimum_required(VERSION 2.8.12)
project(quiz4_a2)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg)

## Declare a catkin package
catkin_package()

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

add_library(analysis src/analysis.cpp)

add_executable(${PROJECT_NAME}_listener src/listener.cpp)
target_link_libraries(${PROJECT_NAME}_listener ${catkin_LIBRARIES} analysis)

##################################################################################################
# The below is our an example of our automated tets for marking
# We have only provided one example, whereas on submission we will test every function you develop
# To enable testsing you need to pass a switch to cmake
# cmake .. -DBUILD_TESTS=ON
# then run make as usual
# finally, tests can be run from build directory (for this example ./test/utest
#if(BUILD_TESTS)
    add_subdirectory(test)
    enable_testing()
#   endif()
