#include "gtest/gtest.h"
#include <vector>
#include <algorithm>

//Student defined libraries
#include "ackerman.h"
#include "pfms_types.h"
#include <cmath>


TEST(ConstructorTest, Ackerman) {
    //Create a ackerman and push back to controllers
    std::vector<ControllerInterface*> controllers;
    controllers.push_back(new Ackerman());

   //Now that we have an Ackerman class, if we call the functions in Ackerman what should they return?
   //What should functions that take no arguments return?


}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
