#include "gtest/gtest.h"
#include <iostream> //Not needed usually, here just for debugging
#include <vector>
#include <algorithm>

//header files needed from our libraries
#include "../rectangle.h"
#include "../processing.h"

using namespace std;

TEST (ClassTest, AnotherConstuctor) {
    Rectangle rectangle(3.0,4.0);
    ASSERT_EQ(rectangle.getDescription(),"rectangle");
}

TEST (ClassTest, CreateObject) {
    Rectangle rectangle;
    rectangle.setHeightWidth(3.0,4.0);
    ASSERT_EQ(rectangle.getDescription(),"rectangle");
}

TEST(ShapeTest, RemoveLargerThanAreaTest) {
  std::vector<Shape*> shapes;
  Rectangle r1(2, 3);
  Rectangle r2(3, 4);
  shapes.push_back(&r1);
  shapes.push_back(&r2);

  double limit = 10.0;
  removeLargerThanArea(shapes, limit);

  ASSERT_EQ(shapes.size(), 1);
  ASSERT_EQ(shapes[0]->getDescription(), "rectangle");
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}

