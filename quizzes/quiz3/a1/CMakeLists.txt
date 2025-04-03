# The minimum version of CMake Required
cmake_minimum_required (VERSION 3.0.0)

# Any project name will suffice, this has connotaions when using advanced CMake Features
project (quiz3_a1)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(BUILD_TESTS ON) # This switch here sets the tests on automatically

find_package( OpenCV REQUIRED )

#################################################################
if(OpenCV_VERSION_MAJOR LESS 4)
    message ( STATUS "OpenCV version 3 " )
    set (RELEASE_CODENAME "opencv3")
else()
    message ( STATUS "OpenCV version 4 " )
    set (RELEASE_CODENAME "opencv4")
endif()

EXECUTE_PROCESS( COMMAND uname -m COMMAND tr -d '\n' OUTPUT_VARIABLE ARCHITECTURE )
if( ${ARCHITECTURE} STREQUAL "aarch64" )
    set (RELEASE_CODENAME "opencv4_arm64")
endif()
###################################################################

# Finding and linking against the simulator library
set(SIMULATOR_LIB ${CMAKE_SOURCE_DIR}/dep/lib/${RELEASE_CODENAME})
set(SIMULATOR_INCLUDE ${CMAKE_SOURCE_DIR}/dep/include)

include_directories(
  ${OpenCV_INCLUDE_DIRS}
  ${SIMULATOR_INCLUDE}
)

message ( STATUS "Looking for libray in ${SIMULATOR_LIB}" )
link_directories(${SIMULATOR_LIB})
#################################################################


# The library students are developing, the one that handles racing
add_library(tf2 tf2.cpp)
target_link_libraries(tf2 simulator )
add_library(analysis analysis.cpp)
target_link_libraries(analysis simulator )

# A sample executable
add_executable(${PROJECT_NAME}_ex main.cpp)
target_link_libraries( ${PROJECT_NAME}_ex analysis tf2 simulator ${OpenCV_LIBS})

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
