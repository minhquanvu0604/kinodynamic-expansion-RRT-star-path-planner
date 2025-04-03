# The minimum version of CMake Required 
cmake_minimum_required (VERSION 2.6)

# Any project name will suffice, this has connotaions when using advanced CMake Features
project (librarytest)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

# If your library is not in the system, state where the include directories are located
# 1. Below will look for them in source directory and lib subfloder
# include_directories( ${CMAKE_SOURCE_DIR}/lib )
# 2. This would look for them in /home/studet/shapes_library/include
#include_directories( $ENV{HOME}/shapes/include )

# If your library is not in the system, state where the include directories are located
# 1. Below will look for them in source directory and lib subfloder
# link_directories( ${CMAKE_SOURCE_DIR}/lib )
# 2. This would look for them in /home/studet/shapes_library/lib
#link_directories( $ENV{HOME}/shapes/lib )

# To add executables provide the executable name followed by a list of source files
# There must be exactly one function called main in each executable

add_executable(${PROJECT_NAME}-ex main.cpp shapeprocessing.cpp)

#link the library (here we called it shapes, you do not need to specify the lib (library would be called libshapes)
target_link_libraries(${PROJECT_NAME}-ex shapes)
