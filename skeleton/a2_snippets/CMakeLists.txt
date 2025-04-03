# The minimum version of CMake Required 
cmake_minimum_required (VERSION 3.0.0)

# Change project name
project (a2_snippets)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

## Include the location of pfms headers
find_package(pipes)

# We create executable from the control_ugv.cpp
add_executable(control_ugv control_ugv.cpp)

# Link the library (needs both pipes and audi, audi is part of pipes from version 2.5.0
target_link_libraries(control_ugv pipes audi)

