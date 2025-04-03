#include "gtest/gtest.h"

#include <vector>
#include <algorithm>

//header files needed from our libraries
#include "../container_ops.h"


TEST (FunctionsTest, ModifyingFrontOfDeque) {
    std::deque<double> container={-0.612444,3.03659,23.678,6.34389,15.0037,-6.98407,-9.78041,6.76262,6.69126,-3.38297};
    std::deque<double> original = container;
    populateContainer(container, 5, -3.0);
    std::deque<double> elements(5,-3.0); //Make a deque of length 5, filled with -3.0

    ASSERT_EQ(container.size(),original.size()+5);// Need asserts here as incorrect size of deque will segfaut below
    ASSERT_TRUE( std::equal(container.begin(), container.begin()+5, elements.begin()) );
    ASSERT_TRUE( std::equal(container.begin()+5, container.end(), original.begin()) );
}


int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
