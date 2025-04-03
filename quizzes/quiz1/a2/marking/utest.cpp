#include "gtest/gtest.h"
#include <iostream> //Not needed usually, here just for debugging
#include <vector>
#include <algorithm>

//header files needed from our libraries
#include "../rectangle.h"
#include "../processing.h"

using namespace std;

TEST (ClassTest, CreateObject) {
    Rectangle rectangle(3.0,4.0);
    ASSERT_EQ(rectangle.getDescription(),"rectangle");
    ASSERT_DOUBLE_EQ(rectangle.getArea(),12.0);

    Rectangle square(3.0,3.0);
    ASSERT_EQ(square.getDescription(),"square");
    ASSERT_DOUBLE_EQ(square.getArea(),9.0);

}


int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
