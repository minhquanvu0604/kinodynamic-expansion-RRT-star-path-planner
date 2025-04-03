# The minimum version of CMake Required 
cmake_minimum_required (VERSION 2.8.13)

# Any project name will suffice, this has connotaions when using advanced CMake Features
project (threading_masterclass)

find_package (Threads)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11  -pthread")

# To add executables provide the executable name followed by a list of source files
# There must be exactly one function called main in each executable
#add_executable(hello hello.cpp)

add_executable(radar_analysis main.cpp radar.cpp dataprocessing.cpp)
