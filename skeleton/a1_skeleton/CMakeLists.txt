# The minimum version of CMake Required 
cmake_minimum_required (VERSION 2.8.13)

# Change project name
project (a1_skeleton)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

set(BUILD_TESTS ON)

find_package(pipes)

# CMake will create your library, add the cpp files needed
add_library(controller controller.cpp ackerman.cpp)
target_link_libraries(controller pipes)

# CMake will create the cell library that you will need 
add_library(mission mission.cpp)

# CMake will create executable add the cpp files needed
add_executable(a1 main.cpp)

# Cmake will link against the libraries in final step
target_link_libraries(a1 controller mission)

if(BUILD_TESTS)
   add_subdirectory(test)
endif()
