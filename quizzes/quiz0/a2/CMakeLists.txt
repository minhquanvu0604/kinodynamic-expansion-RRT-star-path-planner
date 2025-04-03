# The minimum version of CMake Required 
cmake_minimum_required (VERSION 3.0)

# Any project name will suffice, this has connotaions when using advanced CMake Features
project (quiz0_a2)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(BUILD_TESTS ON) # This switch here turns on testing automatically

# To add executables provide the executable name followed by a list of source files
# There must be exactly one function called main in each executable
add_executable(sample_ex main.cpp sample.cpp)

##################################################################################################
# The below enables our automated tets so you can develop/test code
#
# We have only provided two tests, whereas on submission we will test all tasks your developing
# compiled tests can be run from build directory (for this example ./test/utest or selecting utest as executable in IDE
if(BUILD_TESTS)
    add_subdirectory(test)
endif()

