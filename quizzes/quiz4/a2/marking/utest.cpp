#include "gtest/gtest.h"
#include <iostream> //Not needed usually, here just for debugging
#include <vector>

//header files needed from our libraries
#include "../src/analysis.h"
using namespace std;


TEST (AnalysisTest, CountCharecters) {


  unsigned int count = analysis::countCharacters("Hello class 41012");
  EXPECT_EQ(count,17);

}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
