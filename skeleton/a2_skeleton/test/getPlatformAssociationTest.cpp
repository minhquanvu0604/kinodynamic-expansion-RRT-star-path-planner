#include "gtest/gtest.h"
#include <iostream>
#include <vector>
#include "pfms_types.h"
#include "supersearch.h"
#include "ackermansearch.h"
#include "quadcoptersearch.h"

#include "ackerman.h"
#include "quadcopter.h"
#include "mission.h"

// Some helper header for assembling messages and testing
#include "test_helper.h"
#include "linkcommand.h"

TEST(BasicMode, OnlyTest){

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


    std::vector<pfms::geometry_msgs::Point> goalsQuad;
    goalsQuad.push_back({ 5, -2});
    goalsQuad.push_back({ 5, -6});
    goalsQuad.push_back({ 0, -6});


    // We now have controller and goals, let's set up mission
    Mission mission(controllers);
    mission.setMissionObjective(mission::Objective::BASIC);
    mission.setGoals(goalsAck,pfms::PlatformType::ACKERMAN);
    mission.setGoals(goalsQuad,pfms::PlatformType::QUADCOPTER);

    auto result = mission.getPlatformGoalAssociation();

    // Print out the result
    std::cout << "Platform association:" << std::endl;
    for (const auto& p : result) {
        std::cout << "(" << p.first << ", " << p.second << ")" << std::endl;
    }
}

TEST(AdvancedMode, OnlyTest){

    //The below teleports the platforms to starting location
    LinkCommand* linkCommand = new LinkCommand();
    {
        Odometry odo = populateOdoUGV(0,2,0);
        linkCommand->writeCommand(odo);
        odo = populateOdoUAV(0,5,2,0);
        linkCommand->writeCommand(odo);
    }

    std::vector<ControllerInterface*> controllers;
    controllers.push_back(new Ackerman());
    controllers.push_back(new Quadcopter());
    controllers.front()->setTolerance(0.5);

    std::vector<pfms::geometry_msgs::Point> goalsAck = {
        {0, 10}, {0, 60}, {0, 50}, {0, 40}, {0, 30}, {0, 20}
    };

    std::vector<pfms::geometry_msgs::Point> goalsQuad = {
        {0, 5}, {10, -2}, {10, 6}, {0, -2}, {-3, -20}, {-10, -20}
    };

    // We now have controller and goals, let's set up mission
    // Ackerman: 0 5 4 3 2 1
    // Quadcopter: 0 2 1 3 4 5
    Mission mission(controllers);
    mission.setMissionObjective(mission::Objective::ADVANCED);
    mission.setGoals(goalsAck,pfms::PlatformType::ACKERMAN);
    mission.setGoals(goalsQuad,pfms::PlatformType::QUADCOPTER);

    auto result = mission.getPlatformGoalAssociation();

    // Print out the result
    std::cout << "Platform association:" << std::endl;
    for (const auto& p : result) {
        std::cout << "(" << p.first << ", " << p.second << ")" << std::endl;
    }
}

TEST(SuperMode, OnlyTest){

    //The below teleports the platforms to starting location
    LinkCommand* linkCommand = new LinkCommand();
    {
        Odometry odo = populateOdoUGV(100,100,0);
        linkCommand->writeCommand(odo);
        odo = populateOdoUAV(-100,-100,2,0);
        linkCommand->writeCommand(odo);
    }

    std::vector<ControllerInterface*> controllers;
    controllers.push_back(new Ackerman());
    controllers.push_back(new Quadcopter());
    controllers.front()->setTolerance(0.5);

    // It is obvious to find the sorted goals of ackerman 
    // This should returns:
    // Ackerman: 0 1 2
    // Quadcopter: 4 5 3
    std::vector<pfms::geometry_msgs::Point> goals = {
        {50,100}, {100,50}, {50, 50},{-50,-100}, {-100,-50}, {-50, -50}
    };

    // We now have controller and goals, let's set up mission
    Mission mission(controllers);
    mission.setMissionObjective(mission::Objective::SUPER);
    mission.setGoals(goals,pfms::PlatformType::ACKERMAN);
    // mission.setGoals(goalsQuad,pfms::PlatformType::QUADCOPTER);

    auto result = mission.getPlatformGoalAssociation();

    // Print out the result
    std::cout << "Platform association:" << std::endl;
    for (const auto& p : result) {
        std::cout << "(" << p.first << ", " << p.second << ")" << std::endl;
    }
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
