# The minimum version of CMake Required 
cmake_minimum_required (VERSION 2.13)

# Any ppoject name will suffice, this has connotaions when using advanced CMake Features
project (quiz1_a2)

# CMake will now create the Makefiles that are needed to make an executable "hello" that requires "hello.cpp" 
add_executable(quiz1_a2 main.cpp shape.cpp rectangle.cpp processing.cpp)

##################################################################################################
# The below is our an example of our automated tets for marking
# Refer canvas Week 04 - Unit Tests to install the unit testing framework if you wish
#
# We have only provided one example, whereas on submission we will test every function you develop
# To enable testsing you need to pass a switch to cmake
# cmake .. -DBUILD_TESTS=ON
# then run make as usual
# finally, tests can be run from build directory (for this example ./test/utest
if(BUILD_TESTS)
    add_subdirectory(test)
endif()
