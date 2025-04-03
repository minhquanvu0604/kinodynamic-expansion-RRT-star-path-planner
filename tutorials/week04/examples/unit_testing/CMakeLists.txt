cmake_minimum_required(VERSION 2.8)

# Make PROJECT_SOURCE_DIR, PROJECT_BINARY_DIR, and PROJECT_NAME ava ilable.
project(myproject)

# Define our library
add_library(project1 project1.cpp)

# Our testing is in another directory
add_subdirectory(test)
enable_testing() # We have to add this here to enable make tests


