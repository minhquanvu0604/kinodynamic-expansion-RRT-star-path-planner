#include "gtest/gtest.h"
#include <iostream>
#include <vector>
#include <algorithm>
#include <cmath>

#include "pfms_types.h"
//Student defined libraries
#include "ackerman.h"
#include "mission.h"

// Some helper header for assembling messages and testing
#include "a1_test_helper.h"
#include "mission_test.h"

using namespace std;
using namespace pfms::nav_msgs;



///////////////////////////////////////////////////////////
// Unit Tests Start HERE
////////////////////////////////////////////////////////

TEST_F(MissionTest, Simple) {

    {
        Odometry odo = populateOdoUGV(0,2,0);//x=0, y=2, yaw=0;
        pipesFakeOdo_->writeCommand(odo);;// We send fake data
    }

    //Set-up the controllers

    std::vector<ControllerInterface*> controllers;
    controllers.push_back(new Ackerman());

    //Setting tolerance to reach goals
    controllers.at(0)->setTolerance(0.5);

    //Goals
    pfms::geometry_msgs::Point goal0{0,3};
    pfms::geometry_msgs::Point goal1{10,4};

    std::vector<pfms::geometry_msgs::Point*> goals;
    goals.push_back(&goal0);
    goals.push_back(&goal1);

    //Below now is an analysis of the platforms for each goal
    //Ackerman can not reach goal 0
    {
        bool reachable = controllers.at(0)->setGoal(goal0);
        EXPECT_FALSE(reachable);
    }

    //The Ackerman can reach goal 1
    {
        bool reachable = controllers.at(0)->setGoal(goal1);
        double dist = controllers.at(0)->distanceToGoal();
        double t = controllers.at(0)->timeToGoal();
        std::cout << "Ackerman: can reach goal " <<
                             dist << "[m] " << t << "[s]" << std::endl;
        ASSERT_TRUE(reachable);
        EXPECT_NEAR(dist,10.2642,0.2);
        EXPECT_NEAR(t,3.53,0.2);
    }


    //The Ackerman can reach goal 0 from goal 1
    {
        Odometry odo = populateOdoUGV(goal1.x,goal1.y,-M_PI/6);
        pipesFakeOdo_->writeCommand(odo);
        pipesFakeOdo_->writeCommand(odo);

        double distance =0; double t=0;
        pfms::nav_msgs::Odometry estimateGoalPose;
        // Setting tolerance for goal (needed to unit test)
        bool reachable = controllers.at(0)->checkOriginToDestination(odo, goal0,distance,t,estimateGoalPose);

        // MY VERSION TO TEST 
        // bool reachable = controllers.at(0)->setGoal(goal0);
        // distance = controllers.at(0)->distanceToGoal();
        // t = controllers.at(0)->timeToGoal();

        std::cout << "Ackerman: can reach goal 0 from goal 1 " <<
                             distance << "[m] " << t << "[s]" << std::endl;
        ASSERT_TRUE(reachable);
        EXPECT_NEAR(distance,43.36,0.2);
        EXPECT_NEAR(t,14.95,0.5);

        EXPECT_NEAR(estimateGoalPose.position.x,goal0.x,0.2);
        EXPECT_NEAR(estimateGoalPose.position.y,goal0.y,0.2);

        double yaw =estimateGoalPose.yaw;

        // std::cout << "Estimated yaw:" <<  yaw << std::endl;

        if(yaw > M_PI){
            yaw = -((2*M_PI)-yaw);
        }

        // std::cout << "Estimated yaw:" <<  yaw << std::endl;

        //Might need to wrap angle from -PI to PI
        EXPECT_NEAR(estimateGoalPose.yaw,0.723,M_PI/64);
    }

}


int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
