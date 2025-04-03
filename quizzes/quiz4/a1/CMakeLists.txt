# The minimum version of CMake Required
cmake_minimum_required (VERSION 3.0.1)

project (quiz4_a1)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(BUILD_TESTS ON) # This switch here sets the tests ON automatically

find_package(pipes)

#################################################################
# The library we are developing will be called controllers
add_library(controllers controller.cpp quadcopter.cpp)
# We need to link it against pipes, because we use pipes
target_link_libraries(controllers pipes )

# The executable will be called quiz4_a1_ex (as it substitutes PROJECT_NAME from line 4 of the CMakeLists.txt
# We only use main.cpp for the executable
add_executable(${PROJECT_NAME}_ex main.cpp)

# We now need to link the executable againts the library, we don't use pipes directly in the main
target_link_libraries( ${PROJECT_NAME}_ex controllers)

#######################################
# The below is our an example of our automated tets for marking
# tests can be run from build directory (for this example ./test/utest)
if(BUILD_TESTS)
    add_subdirectory(test)
    enable_testing()
endif()
