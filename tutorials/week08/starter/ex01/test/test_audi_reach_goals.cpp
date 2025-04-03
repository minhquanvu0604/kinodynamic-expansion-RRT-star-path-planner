#include "gtest/gtest.h"
#include <iostream>
#include <vector>
#include <algorithm>

//Student defined libraries
#include "ackerman.h"
#include "mission.h"

#include "pfms_types.h"
#include <cmath>
// Some helper header for assembling messages and testing
#include "a1_test_helper.h"
#include "ackerman_test.h"
#include "linkcommand.h"
using namespace std;
using namespace pfms::nav_msgs;


///////////////////////////////////////////////////////////
// Unit Tests Start HERE
////////////////////////////////////////////////////////

TEST(AckermanExTest, FirstGoal) {

    LinkCommand* linkCommand = new LinkCommand(true);
    {
        Odometry odo = populateOdoUGV(0,2,0);
        linkCommand->writeCommand(odo);
    }

    std::vector<ControllerInterface*> controllers;
    controllers.push_back(new Ackerman());

    //Goal at x=10,y=0;
    pfms::geometry_msgs::Point pt{10,0};
    linkCommand->writeCommand(pt,pfms::PlatformType::ACKERMAN);

    controllers.front()->setTolerance(0.5);

    bool reachable = controllers.at(0)->setGoal(pt);
    double dist = controllers.at(0)->distanceToGoal();
    double t = controllers.at(0)->timeToGoal();
    std::cout << "Ackerman: can reach goal " <<
                         dist << "[m] " << t << "[s]" << std::endl;

    ASSERT_TRUE(reachable);

    bool reached = controllers.at(0)->reachGoal();

    ASSERT_TRUE(reached);


    double distance = linkCommand->checkGoalDistance(pfms::PlatformType::ACKERMAN);
    std::cout << "Distance to goal:" << distance << std::endl;

    ASSERT_NEAR(distance,0,1.0);

}

TEST(AckermanExTest, 2ndGoal) {

    //! @todo
    //! Create another test for a second goal.

}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
