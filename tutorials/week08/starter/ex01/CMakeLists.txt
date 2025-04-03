# The minimum version of CMake Required 
cmake_minimum_required (VERSION 3.0.1)

# Add project name
project (week08)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

# To enable testsing we instaed set BUILD_TEST to ON by default
set(BUILD_TESTS ON )

## Include the location of pfms headers
find_package(pipes)

#################################################################
# The below finds version of example library we will test
include(${CMAKE_CURRENT_SOURCE_DIR}/dep/CMakeLists.txt)
###################################################################

add_executable(ex01 main.cpp)
target_link_libraries(ex01 pipes example)

##################################################################################################
# The below enables tests
# We have only provided 3 examples (one is the existing unit test we supplied)
if(BUILD_TESTS)
    add_subdirectory(test)
    enable_testing()
endif()
