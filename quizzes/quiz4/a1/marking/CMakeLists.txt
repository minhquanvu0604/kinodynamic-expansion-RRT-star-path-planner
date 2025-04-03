#######################################################################
# Now we enable gtests - this section does not need to change
set(GTEST_ROOT "/usr/src/gtest" CACHE PATH "Path to googletest")
find_package(GTest REQUIRED)

message(STATUS "The include for simulator is ${SIMULATOR_INCLUDE}")


include_directories(
 ${GTEST_INCLUDE_DIRS}
 ${CMAKE_SOURCE_DIR}  #This enabled us to include files that are part of project
)

if(NOT GTEST_LIBRARY)
   message("GTest library not found")
endif()

enable_testing()
###################################

# Add the unit test executable against the source code of the test
add_executable(selfTest utest.cpp)
# Linking to gtest and to your libraries
target_link_libraries(selfTest GTest::GTest GTest::Main linkcommand controllers pipes)
# We can finally add the test to a list of tests to be run with make test
add_test(Q4A1 selfTest)

# Add the unit test executable against the source code of the test
add_executable(markingTest utest2.cpp)
# Linking to gtest and to your libraries
target_link_libraries(markingTest GTest::GTest GTest::Main linkcommand controllers pipes)
# We can finally add the test to a list of tests to be run with make test
add_test(Q4A1 markingTest)

