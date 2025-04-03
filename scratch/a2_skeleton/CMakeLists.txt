# The minimum version of CMake Required 
cmake_minimum_required (VERSION 3.0.0)

# Change project name
project (a2_skeleton)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread")

set(BUILD_TESTS ON)

find_package (Threads)
find_package(pipes)

# We supply a logger library to read/write Point3d to a file
add_library(logger logger.cpp)

add_library(controller controller.cpp ackerman.cpp quadcopter.cpp)
add_library(mission mission.cpp)
target_link_libraries(controller pipes)
target_link_libraries(mission pipes)

# Create executable with  cpp files needed
add_executable(assignment2 main.cpp)

# Cmake will link against the libraries in final step
target_link_libraries(assignment2 logger controller mission pipes)

if(BUILD_TESTS)
   add_subdirectory(test)
endif()
