# The minimum version of CMake Required 
cmake_minimum_required (VERSION 2.8.13)

# Any project name will suffice, this has connotaions when using advanced CMake Features
project (week07_ex03)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread")

# We make a single executable, but could have made at least one library (or used a radar library)
add_executable(radar_processing main.cpp radar.cpp dataprocessing.cpp)
