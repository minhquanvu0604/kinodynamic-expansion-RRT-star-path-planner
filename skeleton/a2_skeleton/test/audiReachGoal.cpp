#include "gtest/gtest.h"
#include <iostream>
#include <vector>
#include <algorithm>
#include <thread>
#include <cmath>
#include "pfms_types.h"

//Student defined libraries
#include "ackerman.h"
#include "mission.h"

// Some helper header for assembling messages and testing
#include "test_helper.h"
#include "linkcommand.h"

using namespace std;
using namespace pfms::nav_msgs;


///////////////////////////////////////////////////////////
// Unit Tests Start HERE
////////////////////////////////////////////////////////

TEST(AckermanReachGoal, SingleGoal) {

    LinkCommand* linkCommand = new LinkCommand(true);
    {
        Odometry odo = populateOdoUGV(0,2,0);
        linkCommand->writeCommand(odo);
    }

    std::vector<ControllerInterface*> controllers;
    controllers.push_back(new Ackerman());
    controllers.front()->setTolerance(0.5);

    pfms::geometry_msgs::Point pt{10,0};

    std::vector<pfms::geometry_msgs::Point> goals;
    goals.push_back(pt);

    bool reachable = controllers.front()->setGoals(goals);
    double dist = controllers.front()->distanceToGoal();
    double t = controllers.front()->timeToGoal();



    ASSERT_TRUE(reachable);
    ASSERT_NEAR(dist,10.2646,0.5);

    // We have an estimated time to reach goal, which is conservative usually vehicles reaches it in
    // less time. Nevertheless we could use 3x this time as the max time to reach goal (factor of safety)
    //
    // We will loop until that time and if the goal is not reached until then (or we have status
    // indicating IDLE, we know it has been reached
    auto start_time = std::chrono::system_clock::now();

    // We start now
    controllers.front()->run();
    bool OK =false; // This will indicate if mission is completed
    bool timeExceeded = false; // time exceeded

    //Status should be running now
    pfms::PlatformStatus status = controllers.front()->status();
    ASSERT_EQ(status,pfms::PlatformStatus::RUNNING);

    while (!OK){

        auto current_time = std::chrono::system_clock::now();
        //std::chrono::seconds is integer for some reason, thus duration<double>
        auto time_taken = std::chrono::duration_cast<std::chrono::duration<double>>(current_time - start_time);

        if(time_taken.count()>(3*t)){
            //We have now taken time
            timeExceeded=true;
            OK=true;
        }

        status = controllers.front()->status();

        if(status == pfms::PlatformStatus::IDLE){
            //mission accomplished
            OK=true;
        }

        //Let's slow down this loop to 200ms (5Hz)
        std::this_thread::sleep_for(std::chrono::milliseconds(200));

    }

    ASSERT_FALSE(timeExceeded); // time shoudl not be exceeded
    ASSERT_EQ(status,pfms::PlatformStatus::IDLE);//Platform should be idle

    pfms::nav_msgs::Odometry odo = controllers.front()->getOdometry();
    double distance = pow ( pow(odo.position.x-pt.x,2) + pow(odo.position.y-pt.y,2),0.5);

    ASSERT_NEAR(distance,0,0.7); // We shoudl be within 0.7 of goal
}


int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
