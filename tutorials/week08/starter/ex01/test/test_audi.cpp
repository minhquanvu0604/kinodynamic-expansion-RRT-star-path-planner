#include "gtest/gtest.h"
#include <iostream>
#include <vector>
#include <algorithm>

//Student defined libraries
#include "ackerman.h"
#include "pfms_types.h"
#include <cmath>


// Some helper header for assembling messages and testing
#include "a1_test_helper.h"
#include "ackerman_test.h"

using namespace std;
using namespace pfms::nav_msgs;


///////////////////////////////////////////////////////////
// Unit Tests Start HERE
////////////////////////////////////////////////////////

TEST_F(AckermanTest, Simple) {

    Odometry odo = populateOdoUGV(0,2,0);//x=0, y=2, yaw=0;
    linkCommand_->writeCommand(odo);;// We send fake data

    std::vector<ControllerInterface*> controllers;
    controllers.push_back(new Ackerman());

    //Goal at x=10,y=0;
    pfms::geometry_msgs::Point pt;
    pt.x=10;
    pt.y=0;

    bool reachable = controllers.at(0)->setGoal(pt);
    double dist = controllers.at(0)->distanceToGoal();
    double t = controllers.at(0)->timeToGoal();
    std::cout << "Ackerman: can reach goal " <<
                         dist << "[m] " << t << "[s]" << std::endl;

    ASSERT_TRUE(reachable);
    ASSERT_NEAR(dist,10.2646,0.5);
    ASSERT_NEAR(t,3.53951,1.0);
}


TEST_F(AckermanTest, NotPossible) {

    //! @todo
    //! Create a unit test that fails reaching 
}


TEST_F(AckermanTest, StraightLine) {

    //! @todo
    //! Create a unit test that checks straigh line
}

TEST_F(AckermanTest, Simplt2) {

    //! @todo
    //! Create a unit test that sucsseeds
}



int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
