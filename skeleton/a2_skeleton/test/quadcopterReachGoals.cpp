#include "gtest/gtest.h"
#include <iostream>
#include <vector>
#include <algorithm>
#include <thread>
#include <cmath>
#include "pfms_types.h"

//Student defined libraries
#include "quadcopter.h"
#include "mission.h"

// Some helper header for assembling messages and testing
#include "test_helper.h"
#include "linkcommand.h"
using namespace std;
using namespace pfms::nav_msgs;


///////////////////////////////////////////////////////////
// Unit Tests Start HERE
////////////////////////////////////////////////////////

TEST(QuadcopterReachGoals, TwoGoals) {


    LinkCommand* linkCommand = new LinkCommand(true);
    {
        Odometry odo = populateOdoUAV(0,2,2,0);
        linkCommand->writeCommand(odo);
    }

    std::vector<ControllerInterface*> controllers;
    controllers.push_back(new Quadcopter());
    controllers.front()->setTolerance(0.5);

    std::vector<pfms::geometry_msgs::Point> goals;
    goals.push_back({ 10, -2});
    goals.push_back({ 10,  6});


    // We now have controller and goals, let's set up mission
    Mission mission(controllers);
    mission.setMissionObjective(mission::Objective::BASIC);
    mission.setGoals(goals,pfms::PlatformType::QUADCOPTER);
    bool reachable = mission.run();

    ASSERT_TRUE(reachable);

    // We have an estimated time to reach goal, which is conservative usually vehicles reaches it in
    // less time. Nevertheless we could use 3x this time as the max time to reach goal (factor of safety)
    //
    // We will loop until that time and if the goal is not reached until then (or we have status
    // indicating IDLE, we know it has been reached
    auto start_time = std::chrono::system_clock::now();

    // We start now
    bool OK =false; // This will indicate if mission is completed

    while (!OK){

        std::vector<unsigned int> progress = mission.status();
        // std::cout << "PERCENT" << progress.at(0) << std::endl;

        if(progress.front() == 100){ // we check progess, at 100 mission is acomplished
            //mission accomplished
            OK=true;
        }
        else {
            pfms::PlatformStatus status = controllers.front()->status();
            ASSERT_EQ(status,pfms::PlatformStatus::RUNNING);//Platform should be running as mission has started and not completed
            ASSERT_LT(progress.front(),100);
        }

        //Let's slow down this loop to 200ms (5Hz)
        std::this_thread::sleep_for(std::chrono::milliseconds(200));

    }

    // When we complete last goal, we should be at the position of last goal
    pfms::nav_msgs::Odometry odo = controllers.front()->getOdometry();
    double distance = pow ( pow(odo.position.x-goals.back().x,2) + 
                            pow(odo.position.y-goals.back().y,2),0.5);
    ASSERT_NEAR(distance,0,0.7); // We shoudl be within 0.7 of goal
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
