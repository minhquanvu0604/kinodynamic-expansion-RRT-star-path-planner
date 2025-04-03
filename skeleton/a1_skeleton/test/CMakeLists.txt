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

add_executable(audiTests test_audi.cpp)
target_link_libraries(audiTests ${GTEST_LIBRARIES} linkcommand student_lib pthread)

add_executable(missionTests test_mission.cpp)
target_link_libraries(missionTests ${GTEST_LIBRARIES} linkcommand student_lib pthread)

add_executable(calcTests test_calculations.cpp)
target_link_libraries(calcTests ${GTEST_LIBRARIES} linkcommand student_lib pthread)

add_executable(my_test my_test.cpp)
target_link_libraries(my_test ${GTEST_LIBRARIES} linkcommand student_lib pthread)