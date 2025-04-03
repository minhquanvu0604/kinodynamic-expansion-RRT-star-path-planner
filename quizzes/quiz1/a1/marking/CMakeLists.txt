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
add_library(personMarking ../person.cpp)
add_library(processingMarking ../processing.cpp)

# Add the unit test executable against the source code of the test
add_executable(selfTest utest.cpp)
add_executable(markingTest utest2.cpp)
# Linking to gtest and to your libraries
target_link_libraries(selfTest GTest::GTest GTest::Main personMarking processingMarking)
target_link_libraries(markingTest GTest::GTest GTest::Main personMarking processingMarking)
# We can finally add the test to a list of tests to be run with make test
add_test(Q1A1_1 selfTest)
add_test(Q1A1_2 markingTest)
