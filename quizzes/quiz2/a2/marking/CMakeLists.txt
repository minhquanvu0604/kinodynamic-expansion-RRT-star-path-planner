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

# We make a library in here for now, this should be done in the CMakeLists of the actual library
# But as we have not covered this in class (Libraries are in Week 04) we create library here
add_library(container_ops_Marking ../container_ops.cpp)


# Add the unit test executable against the source code of the test
add_executable(markingTest utest.cpp)
add_executable(selfTest self.cpp)
# Linking to gtest and to your libraries
target_link_libraries(markingTest GTest::GTest GTest::Main container_ops_Marking)
target_link_libraries(selfTest GTest::GTest GTest::Main container_ops_Marking)
# We can finally add the test to a list of tests to be run with make test
add_test(Q2A2 markingTest)
add_test(Q2A2 selfTest)
