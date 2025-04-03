#include "gtest/gtest.h"
#include <iostream> //Not needed usually, here just for debugging
#include <vector>

//header files needed from our libraries
#include "../src/analysis.h"
using namespace std;


TEST (AnalysisTest, DetermineNumber) {

  int num = analysis::getNumber("Hello class 41012 YAY 2021!");
  EXPECT_EQ(num,41012);

}


int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
