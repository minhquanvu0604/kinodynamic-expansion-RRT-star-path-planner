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

include_directories(../) #includes below directory so we don't have to do #include "../project.h"

# Add the unit test executable against the source code of the test
add_executable(unitTests utest.cpp)

# Linking to gtest and to your libraries
target_link_libraries(unitTests GTest::GTest GTest::Main project1)

# We can finally add the test to a list of tests to be run with make test
add_test(WEEK04 unitTests)

