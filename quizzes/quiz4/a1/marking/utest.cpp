#include "gtest/gtest.h"
#include <iostream>
#include <vector>
#include <algorithm>
#include <cmath>

#include "quadcopter.h" // The quadcopter
#include "pfms_types.h" //A1 types
#include "test_helper.h" // Helper header that assembled the message
#include "linkcommand.h" // Controlling the simulator



using namespace pfms::nav_msgs;

TEST(QuadcopterTest, Constructor) {

    LinkCommand* linkCommand = new LinkCommand;
    {
        Odometry odo = populateOdoUAV(2,-2,0,M_PI/4);
        linkCommand->writeCommand(odo);
    }

    //Create a quadcopter and push back to controllers
    std::vector<ControllerInterface*> controllers;
    controllers.push_back(new Quadcopter());
    double tolerance =0.5;
    controllers.front()->setTolerance(tolerance);

    
    EXPECT_FLOAT_EQ(controllers.at(0)->distanceToGoal(),0);//Should be zero
    EXPECT_FLOAT_EQ(controllers.at(0)->timeToGoal(),0); //Should be zero
    EXPECT_FLOAT_EQ(controllers.at(0)->timeInMotion(),0); //Should be zero
    EXPECT_FLOAT_EQ(controllers.at(0)->distanceTravelled(),0); //Should be zero
    EXPECT_EQ(controllers.at(0)->getPlatformType(),pfms::PlatformType::QUADCOPTER);//Quadcopter type

    //If we call getOdometry we expect it to retrun the location we set as initial odometry
    pfms::nav_msgs::Odometry odo = controllers.at(0)->getOdometry();
    EXPECT_NEAR(odo.position.x,2,tolerance);
    EXPECT_NEAR(odo.position.y,-2,tolerance);
}


TEST(QuadcopterTest, Goals) {

    LinkCommand* linkCommand = new LinkCommand;
    {
        Odometry odo = populateOdoUAV(0,0,0,0);
        linkCommand->writeCommand(odo);
    }   

    //Create a quadcopter and push back to controllers
    std::vector<ControllerInterface*> controllers;
    controllers.push_back(new Quadcopter());

    double tolerance =0.2;
    controllers.front()->setTolerance(tolerance);

    // Will give slightly larger tolerance for testing
    tolerance = 0.5;

    // We try to reach fisrt point
    {
        //Goal location
        pfms::geometry_msgs::Point pt{4,1};

        // Set the goal for controller zero (the Quad)
        bool reachable = controllers.at(0)->setGoal(pt);

        // Get distance to goal
        double dist = controllers.at(0)->distanceToGoal();
        // Get time to goal
        double t = controllers.at(0)->timeToGoal();

        std::cout << "Quadcopter: can reach goal " <<
                             dist << "[m] " << t << "[s]" << std::endl;

        ASSERT_TRUE(reachable);// The goal should be reachable
        ASSERT_NEAR(dist,4.123,tolerance);//Distance 4.123[m]
        ASSERT_NEAR(t,10.307,0.5); //Time 3.535 [s], tolerance 0.5 [s] for the answer

        bool reached = controllers.at(0)->reachGoal();

        ASSERT_TRUE(reached);

        pfms::nav_msgs::Odometry odo = controllers.at(0)->getOdometry();
        double distance = pow ( pow(odo.position.x-pt.x,2) + pow(odo.position.y-pt.y,2),0.5);
        std::cout << "Distance to goal:" << distance << std::endl;
        ASSERT_NEAR(distance,0,tolerance);
    }

    //The 2nd goal, relies on fact we reached first, ASSERT will prevent below running
    // If we did not reach first goal.
    {
        pfms::geometry_msgs::Point pt{0,-6.5};

        bool reachable = controllers.at(0)->setGoal(pt);
        double dist = controllers.at(0)->distanceToGoal();
        double t = controllers.at(0)->timeToGoal();
        std::cout << "Quadcopter: can reach goal " <<
                             dist << "[m] " << t << "[s]" << std::endl;

        ASSERT_TRUE(reachable);
        ASSERT_NEAR(dist,8.5,tolerance);
        ASSERT_NEAR(t,20.501,1.0);

        bool reached = controllers.at(0)->reachGoal();

        ASSERT_TRUE(reached);

        pfms::nav_msgs::Odometry odo = controllers.at(0)->getOdometry();
        double distance = pow ( pow(odo.position.x-pt.x,2) + pow(odo.position.y-pt.y,2),0.5);

        std::cout << "Distance to goal:" << distance << std::endl;

        ASSERT_NEAR(distance,0,tolerance);
    }

}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
