#include "gtest/gtest.h"
#include <iostream>
#include <vector>
#include <algorithm>
#include <thread>
#include <cmath>
#include "pfms_types.h"

//Student defined libraries
#include "ackerman.h"
#include "quadcopter.h"
#include "mission.h"

// Some helper header for assembling messages and testing
#include "test_helper.h"
#include "linkcommand.h"

using namespace std;
using namespace pfms::nav_msgs;


///////////////////////////////////////////////////////////
// Unit Tests Start HERE
/////////////////////////////////////////////////////////

TEST(MissionReachGoals, ThreeGoals) {

    //The below teleports the platforms to starting location
    LinkCommand* linkCommand = new LinkCommand();
    {
        Odometry odo = populateOdoUGV(0,2,0);
        linkCommand->writeCommand(odo);
        odo = populateOdoUAV(0,0,0,M_PI/4);
        linkCommand->writeCommand(odo);
    }

    std::vector<ControllerInterface*> controllers;
    controllers.push_back(new Ackerman());
    controllers.push_back(new Quadcopter());
    controllers.front()->setTolerance(0.5);

    std::vector<pfms::geometry_msgs::Point> goalsAck;
    goalsAck.push_back({ 5, 2});
    goalsAck.push_back({ 10, 0});
    goalsAck.push_back({ 20, -20});

    linkCommand->writeCommand(goalsAck,"/reach_ugv/set_goals");

    std::vector<pfms::geometry_msgs::Point> goalsQuad;
    goalsQuad.push_back({ 5, -2});
    goalsQuad.push_back({ 5, -6});
    goalsQuad.push_back({ 0, -6});

    linkCommand->writeCommand(goalsQuad,"/reach_uav/set_goals");

    // We now have controller and goals, let's set up mission
    Mission mission(controllers);
    mission.setMissionObjective(mission::Objective::BASIC);
    mission.setGoals(goalsAck,pfms::PlatformType::ACKERMAN);
    mission.setGoals(goalsQuad,pfms::PlatformType::QUADCOPTER);

    auto start_call_mission_time = std::chrono::system_clock::now();
    // We run mission  (non blocking)
    mission.run();
    auto current_time = std::chrono::system_clock::now();
    auto time_taken = std::chrono::duration_cast<std::chrono::duration<double>>(current_time - start_call_mission_time);

    EXPECT_NEAR(time_taken.count(),0,5.0);//This checks if run mission is a blocking call, should return quickly

    // We have an estimated time to reach goal, which is conservative usually vehicles reaches it in
    // less time. 
    //
    // Just in case students have not implemented this correctly, we have an upper bound of 90s
    // We will loop until that time and if the goal is not reached until then (or we have status
    // indicating IDLE, we know it has been reached
    auto start_time = std::chrono::system_clock::now();
    double maxTime =90.0;

    bool OK =false; // This will indicate if mission is completed
    bool timeExceeded = false; // time exceeded

    while (!OK){

        auto current_time = std::chrono::system_clock::now();
        //std::chrono::seconds is integer for some reason, thus duration<double>
        auto time_taken = std::chrono::duration_cast<std::chrono::duration<double>>(current_time - start_time);

        if(time_taken.count()>(maxTime)){
            //We have now taken time
            timeExceeded=true;
            OK=true;
        }

        std::vector<unsigned int> status = mission.status();

        ASSERT_EQ(status.size(),2);

        // std::cout << "progress ..." << status.front() << "% " <<  status.back() << "% " << std::endl;
        // std::this_thread::sleep_for(std::chrono::milliseconds(200));

        bool completed = true;
        for (auto st:status){
            if(st < 100){
                //mission accomplished
                completed=false;
            }
        }

        if(completed){
            OK=true;
        }
        else{
            //Let's slow down this loop to 200ms (5Hz)
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
        }
    }

    ASSERT_FALSE(timeExceeded); // time should not be exceeded

    bool reachedUAV = linkCommand->checkGoalsReached("/reach_uav/check_goals");
    bool reachedUGV = linkCommand->checkGoalsReached("/reach_ugv/check_goals");

    ASSERT_TRUE(reachedUAV); // All goals reached within 0.5m
    ASSERT_TRUE(reachedUGV); // All goals reached within 0.5m
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
