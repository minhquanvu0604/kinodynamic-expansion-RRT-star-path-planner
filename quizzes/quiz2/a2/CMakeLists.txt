cmake_minimum_required (VERSION 3.1)

project(quiz2_a2)
set(CMAKE_CXX_STANDARD 11)
set(BUILD_TESTS ON)

add_executable(${PROJECT_NAME}_ex main.cpp container_ops.cpp)

##################################################################################################
# The below is our an example of our automated tets for marking
# We have only provided one example, whereas on submission we will test every function you develop
# To enable testsing you need to pass a switch to cmake
# cmake .. -DBUILD_TESTS=ON
# then run make as usual
# finally, tests can be run from build directory (for this example ./test/utest
if(BUILD_TESTS)
    add_subdirectory(test)
    enable_testing()
endif()

if(MARKING_TESTS)
    enable_testing()
    add_subdirectory(marking)
endif()
