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

TEST(QuadcopterTest, TotalDistance) {

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

    //Goal location
    pfms::geometry_msgs::Point pt1{4,1};
    pfms::geometry_msgs::Point pt2{2,-2.5};

    double distance1= std::pow ( std::pow(pt1.x-0,2)+ std::pow(pt1.y-0,2),0.5);
    double distance2= std::pow ( std::pow(pt2.x-pt1.x,2)+ std::pow(pt2.y-pt1.y,2),0.5);

    // We try to reach fisrt point
    {
        // Set the goal for controller zero (the Quad)
        bool reachable = controllers.at(0)->setGoal(pt1);

        // Get distance to goal
        double dist = controllers.at(0)->distanceToGoal();
        // Get time to goal
        double t = controllers.at(0)->timeToGoal();

        std::cout << "Quadcopter: can reach goal " <<
                             dist << "[m] " << t << "[s]" << std::endl;


        ASSERT_TRUE(reachable);// The goal should be reachable
        ASSERT_NEAR(dist,distance1,tolerance);
        ASSERT_NEAR(t,distance1/0.4,1.0);

        bool reached = controllers.at(0)->reachGoal();
        ASSERT_TRUE(reached);
    }

    //The 2nd goal, relies on fact we reached first, ASSERT will prevent below running
    // If we did not reach first goal.
    {

        bool reachable = controllers.at(0)->setGoal(pt2);
        double dist = controllers.at(0)->distanceToGoal();
        double t = controllers.at(0)->timeToGoal();
        std::cout << "Quadcopter: can reach goal " <<
                             dist << "[m] " << t << "[s]" << std::endl;

        ASSERT_TRUE(reachable);
        ASSERT_NEAR(dist,distance2,tolerance);
        ASSERT_NEAR(t,distance2/0.4,1.0);

        bool reached = controllers.at(0)->reachGoal();

        ASSERT_TRUE(reached);

    }

    ASSERT_NEAR(controllers.at(0)->distanceTravelled(),distance1+distance2,1.0);

}

TEST(QuadcopterTest, TotalTime) {

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

    //Goal location
    pfms::geometry_msgs::Point pt1{2,1.2};
    pfms::geometry_msgs::Point pt2{0,-2.0};

    double distance1= std::pow ( std::pow(pt1.x-0,2)+ std::pow(pt1.y-0,2),0.5);
    double distance2= std::pow ( std::pow(pt2.x-pt1.x,2)+ std::pow(pt2.y-pt1.y,2),0.5);

    // We try to reach fisrt point
    {
        // Set the goal for controller zero (the Quad)
        bool reachable = controllers.at(0)->setGoal(pt1);

        // Get distance to goal
        double dist = controllers.at(0)->distanceToGoal();
        // Get time to goal
        double t = controllers.at(0)->timeToGoal();

        std::cout << "Quadcopter: can reach goal " <<
                             dist << "[m] " << t << "[s]" << std::endl;


        ASSERT_TRUE(reachable);// The goal should be reachable
        ASSERT_NEAR(dist,distance1,tolerance);
        ASSERT_NEAR(t,distance1/0.4,1.0);

        bool reached = controllers.at(0)->reachGoal();
        ASSERT_TRUE(reached);
    }

    //The 2nd goal, relies on fact we reached first, ASSERT will prevent below running
    // If we did not reach first goal.
    {

        bool reachable = controllers.at(0)->setGoal(pt2);
        double dist = controllers.at(0)->distanceToGoal();
        double t = controllers.at(0)->timeToGoal();
        std::cout << "Quadcopter: can reach goal " <<
                             dist << "[m] " << t << "[s]" << std::endl;

        ASSERT_TRUE(reachable);
        ASSERT_NEAR(dist,distance2,tolerance);
        ASSERT_NEAR(t,distance2/0.4,1.0);

        bool reached = controllers.at(0)->reachGoal();

        ASSERT_TRUE(reached);

    }

    double anticipatedTime = (distance1+distance2)/0.4;
    double toleranceTime = anticipatedTime*0.1; // 10% error accepted

    ASSERT_NEAR(controllers.at(0)->timeInMotion(),anticipatedTime,toleranceTime);

}



int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
