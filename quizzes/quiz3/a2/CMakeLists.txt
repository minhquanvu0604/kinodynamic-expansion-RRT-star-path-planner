# The minimum version of CMake Required
cmake_minimum_required (VERSION 2.6)

# Any project name will suffice, this has connotaions when using advanced CMake Features
project (quiz3_a2)
set(BUILD_TESTS ON)

# Needed to use threads
find_package (Threads)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -no-pie -pthread")

#################################################################
# The below finds version of library based on your system
execute_process(COMMAND lsb_release -cs
    OUTPUT_VARIABLE RELEASE_CODENAME
    OUTPUT_STRIP_TRAILING_WHITESPACE
)

if("${RELEASE_CODENAME}" STREQUAL "bionic")
    message ( STATUS "Ubuntu is in use, LTS version ${RELEASE_CODENAME}" )
elseif("${RELEASE_CODENAME}" STREQUAL "focal")
    message ( STATUS "Ubuntu is in use, LTS version ${RELEASE_CODENAME}" )
else("${RELEASE_CODENAME}" STREQUAL "bionic")
    message ( FATAL_ERROR "Your vesrion on Ubuntu ${RELEASE_CODENAME} is not supported" )
endif("${RELEASE_CODENAME}" STREQUAL "bionic")

EXECUTE_PROCESS( COMMAND uname -m COMMAND tr -d '\n' OUTPUT_VARIABLE ARCHITECTURE )
if( ${ARCHITECTURE} STREQUAL "aarch64" )
    set (RELEASE_CODENAME "arm64")
endif()

# Find the library and files belonging to Radar (they have been provided in the dep subdirectory)
set(RADAR_LIB_HOME ${CMAKE_SOURCE_DIR}/dep)
include_directories(${RADAR_LIB_HOME}/include)
link_directories(${RADAR_LIB_HOME}/lib/${RELEASE_CODENAME})
###################################################################

add_library(analysis analysis.cpp)
target_link_libraries(analysis radar)

# Add executable
add_executable(${PROJECT_NAME}_ex main.cpp)

# Link the library
target_link_libraries(${PROJECT_NAME}_ex analysis)

# tests can be run from build directory (for this example ./test/utest
if(BUILD_TESTS)
    add_subdirectory(test)
endif()
