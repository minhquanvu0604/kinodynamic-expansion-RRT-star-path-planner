#include "gtest/gtest.h"
#include <iostream> //Not needed usually, here just for debugging
#include <vector>
#include <algorithm>

//header files needed from our libraries
#include "../sample.h"

using namespace std;


TEST (ClassTest, CreateObject) {
    Sample sample(5.0);
    EXPECT_EQ(sample.readValue(), 5.0);
}


int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
