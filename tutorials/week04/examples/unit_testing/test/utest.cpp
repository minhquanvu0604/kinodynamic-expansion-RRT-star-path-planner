#include "gtest/gtest.h"
#include "project1.h"
#include <cmath>

TEST(SquareRootTest, PositiveNos) { 
    EXPECT_DOUBLE_EQ (18.0, squareroot (324.0));
    EXPECT_DOUBLE_EQ (25.4, squareroot (645.16));
    EXPECT_DOUBLE_EQ (50.332, squareroot (2533.310224));
}

TEST (SquareRootTest, ZeroAndNegativeNos) { 
    ASSERT_EQ (0.0, squareroot (0.0));
    ASSERT_TRUE (std::isnan(squareroot (-22.0)));
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
