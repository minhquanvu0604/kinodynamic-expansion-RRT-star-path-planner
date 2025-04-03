#include "gtest/gtest.h"
#include <iostream> //Not needed usually, here just for debugging
#include <vector>
#include <algorithm>

//header files needed from our libraries
#include "../rectangle.h"
#include "../processing.h"
using std::vector;

TEST (ContainerTest, RemoveElements) {
    vector<Shape *> shapes;
    shapes.push_back(new Rectangle(1.0,4.0));
    shapes.push_back(new Rectangle(3.0,3.0));
    shapes.push_back(new Rectangle(2.0,2.0));
    shapes.push_back(new Rectangle(10.0,1.0));
    removeLargerThanArea(shapes,4.5);

    ASSERT_EQ(shapes.size(),2);
}



int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
