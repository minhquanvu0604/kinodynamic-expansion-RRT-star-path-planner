#######################################################################
# Now we enable gtests - this section does not need to change
set(GTEST_ROOT "/usr/src/gtest" CACHE PATH "Path to googletest")
find_package(GTest REQUIRED)

include_directories(${GTEST_INCLUDE_DIRS})

if(NOT GTEST_LIBRARY)
   message("GTest library not found")
endif()

enable_testing()
###################################

# Just in case someone had the include files in seperate directory
include_directories(../include)
include_directories(..)

# This uses the complete student's library
aux_source_directory(.. SRC_LIST)
list(REMOVE_ITEM SRC_LIST "../main.cpp")
message ( STATUS "Compiling student_lib with following files  ${SRC_LIST}" )
add_library(student_lib ${SRC_LIST})
target_link_libraries(student_lib pipes)

## Three tests for the AUDI
add_executable(audiCheckOriginToDestination audiCheckOriginToDestination.cpp)
target_link_libraries(audiCheckOriginToDestination ${GTEST_LIBRARIES} linkcommand student_lib pthread pipes)

add_executable(audiReachGoal audiReachGoal.cpp)
target_link_libraries(audiReachGoal ${GTEST_LIBRARIES} linkcommand student_lib pthread)

add_executable(audiReachGoals audiReachGoals.cpp)
target_link_libraries(audiReachGoals ${GTEST_LIBRARIES} linkcommand student_lib pthread)

# Three tests for the QUADCOPTER
add_executable(quadcopterCheckOriginToDestination quadcopterCheckOriginToDestination.cpp)
target_link_libraries(quadcopterCheckOriginToDestination ${GTEST_LIBRARIES} student_lib linkcommand  pthread)

add_executable(quadcopterReachGoal quadcopterReachGoal.cpp)
target_link_libraries(quadcopterReachGoal ${GTEST_LIBRARIES} student_lib linkcommand  pthread)

add_executable(quadcopterReachGoals quadcopterReachGoals.cpp)
target_link_libraries(quadcopterReachGoals ${GTEST_LIBRARIES} student_lib linkcommand pthread)

# One test for the MISSION
add_executable(missionExReachGoals missionExReachGoals.cpp)
target_link_libraries(missionExReachGoals ${GTEST_LIBRARIES} student_lib linkcommand pthread)


###################################################################################
# CUSTOM UNIT TESTS

# Graph search testing
add_executable(graphSearchTest graphSearchTest.cpp)
target_link_libraries(graphSearchTest ${GTEST_LIBRARIES} student_lib linkcommand pthread)

# Super mode testing
add_executable(superTest superTest.cpp)
target_link_libraries(superTest ${GTEST_LIBRARIES} student_lib linkcommand pthread)

# Get platform association testing
add_executable(getPlatformAssociationTest getPlatformAssociationTest.cpp)
target_link_libraries(getPlatformAssociationTest ${GTEST_LIBRARIES} student_lib linkcommand pthread)



