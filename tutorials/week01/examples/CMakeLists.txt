# The minimum version of CMake Required 
cmake_minimum_required (VERSION 2.6)

# Any project name will suffice, this has connotaions when using advanced CMake Features
project (week_01_examples)

# The below line will allow using C++11 features
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

# To add executables provide the executable name followed by a list of source files
# There must be exactly one function called main in each executable
add_executable(hello hello.cpp)

add_executable(pointers pointers.cpp)

add_executable(arrays arrays.cpp)

add_executable(c_strings c_strings.cpp)

add_executable(loops_typecasting loops_typecasting.cpp)

add_executable(structs structs.cpp)

add_executable(switch_case switch_case.cpp)

add_executable(functions functions.cpp)

add_executable(vectors vectors.cpp)

add_executable(aircon aircon.cpp)
