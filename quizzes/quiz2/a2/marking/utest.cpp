#include "gtest/gtest.h"

#include <vector>
#include <algorithm>

//header files needed from our libraries
#include "../container_ops.h"

//Only left the additional test here
TEST (FunctionsTest, BubbleSort) {
    std::deque<double> original={-0.612444,3.03659,23.678,6.34389,15.0037,-6.98407,-9.78041,6.76262,6.69126,-3.38297};
    std::deque<double> sorted = {-9.78041,-6.98407,-3.38297,-0.612444,3.03659,6.34389,6.69126,6.76262,15.0037,23.678};

    bubbleSortContainer(original);

    ASSERT_EQ(sorted.size(),original.size());// Need asserts here as incorrect size of deque will segfaut below

    ASSERT_TRUE( std::equal(original.begin(), original.end(), sorted.begin()) );

    //std::swap(original,sorted);
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
