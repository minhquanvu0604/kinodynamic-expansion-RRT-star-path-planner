# The minimum version of CMake Required 
cmake_minimum_required (VERSION 2.8.13)

# Change project name
project (a1_examples)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

## Include the location of pfms headers
find_package(pipes)

# This executable demonstrates the declaration, implementation, and use of the rectangle class
# Notice we now have two source files, one has our main, the other implements our rectangle class
add_executable(command_goals command_goals.cpp)
add_executable(command_ugv command_ugv.cpp)
add_executable(command_uav command_uav.cpp)
add_executable(populate_first populate_first.cpp)
add_executable(populate_back populate_back.cpp)


# Link the library
target_link_libraries(command_goals pipes)
target_link_libraries(command_ugv pipes)
target_link_libraries(command_uav pipes)
target_link_libraries(populate_first linkcommand pipes)
target_link_libraries(populate_back linkcommand pipes)
