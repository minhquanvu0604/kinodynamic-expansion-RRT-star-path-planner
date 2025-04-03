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

include_directories(
 ${CMAKE_SOURCE_DIR}  #This enabled us to include files that are part of project
)

# Add the unit test executable against the source code of the test
add_executable(constructorsTests test_constructors.cpp)
# Linking to gtest and to your libraries
target_link_libraries(constructorsTests GTest::GTest GTest::Main pthread pipes example)
# We can finally add the test to a list of tests to be run with make test
add_test(EX1 constructorsTests)

# There should be equivalence between
# GTest::GTest GTest::Main
# AND
# ${GTEST_LIBRARIES}

# Add the unit test executable against the source code of the test
add_executable(audiTests test_audi.cpp)
# Linking to gtest and to your libraries
target_link_libraries(audiTests GTest::GTest GTest::Main pthread pipes linkcommand example)
# We can finally add the test to a list of tests to be run with make test
add_test(EX1 audiTests)

# Add the unit test executable against the source code of the test
add_executable(audiExGoals test_audi_reach_goals.cpp)
# Linking to gtest and to your libraries
target_link_libraries(audiExGoals ${GTEST_LIBRARIES} pthread pipes linkcommand example )
# We can finally add the test to a list of tests to be run with make test
add_test(EX3 audiExGoals)


