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
    
    //  SCRATCH - DELETE AFTER THIS
    // Odometry odo = populateOdoUGV(0,2,0);//x=0, y=2, yaw=0;
    // pipesFakeOdo_->writeCommand(odo);;// We send fake data

    // std::vector<ControllerInterface*> controllers;
    // controllers.push_back(new Ackerman());

    //////////////////////////////////


    // Send the car anywhere we want - teleporting
    Odometry odo = populateOdoUGV(0,2,0);//x=0, y=2, yaw=0;
    pipesFakeOdo_->writeCommand(odo);;// We send fake data
    // This pipe is created only for testing

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


TEST_F(AckermanTest, Simple2) {

    Odometry odo = populateOdoUGV(0,2,0);//x=0, y=2, yaw=0;
    pipesFakeOdo_->writeCommand(odo);;// We send fake data

    std::vector<ControllerInterface*> controllers;
    controllers.push_back(new Ackerman());

    pfms::geometry_msgs::Point pt;
    pt.x=0;
    pt.y=-6.5;

    bool reachable = controllers.at(0)->setGoal(pt);
    double dist = controllers.at(0)->distanceToGoal();
    double t = controllers.at(0)->timeToGoal();
    std::cout << "Ackerman: can reach goal " <<
                         dist << "[m] " << t << "[s]" << std::endl;

    ASSERT_TRUE(reachable);
    ASSERT_NEAR(dist,13.351,0.5);
    ASSERT_NEAR(t,4.6404,1.0);
}

TEST_F(AckermanTest, NotPossible) {

    Odometry odo = populateOdoUGV(0,2,0);//x=0, y=2, yaw=0;
    pipesFakeOdo_->writeCommand(odo);;// We send fake data

    std::vector<ControllerInterface*> controllers;
    controllers.push_back(new Ackerman());

    //Goal at x=-6,y=0;//This shoudl not be possible, exceeds max driving circle
    pfms::geometry_msgs::Point pt {0,-6};

    bool reachable = controllers.at(0)->setGoal(pt);
    ASSERT_FALSE(reachable);
}


TEST_F(AckermanTest, ForwardFacing) {

    Odometry odo = populateOdoUGV(0.225,-5.306,127.825*M_PI/180);//x=0.225, y=-5.306, yaw=127deg
    pipesFakeOdo_->writeCommand(odo);;// We send fake data

    std::vector<ControllerInterface*> controllers;
    controllers.push_back(new Ackerman());

    //Goal at x=1,y=4;//This shoudl possible
    pfms::geometry_msgs::Point pt {1,4};

    bool reachable = controllers.at(0)->setGoal(pt);
    ASSERT_TRUE(reachable);

    double dist = controllers.at(0)->distanceToGoal();
    double t = controllers.at(0)->timeToGoal();
    std::cout << "Ackerman: can reach goal " <<
                         dist << "[m] " << t << "[s]" << std::endl;

    ASSERT_NEAR(dist,10.2568,0.5);
    ASSERT_NEAR(t,3.53684,1.0);
}


int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
