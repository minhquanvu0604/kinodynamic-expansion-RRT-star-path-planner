#include "gtest/gtest.h"
#include <iostream>
#include <vector>
#include <algorithm>

//Student defined libraries
#include "ackerman.h"
#include "mission.h"

#include "pfms_types.h"
#include "linkcommand.h"
#include "a1_test_helper.h"
#include <cmath>

using std::cout;
using std::endl;
using namespace pfms::nav_msgs;



///////////////////////////////////////////////////////////
// Unit Tests Start HERE
////////////////////////////////////////////////////////

TEST(MissionTest, 1stTest) {

    LinkCommand* linkCommand = new LinkCommand;
    {
        Odometry odo = populateOdoUGV(0,2,0);//x=0, y=2, yaw=0;
        linkCommand->writeCommand(odo);;// We send fake data
    }
    //Set-up the controllers
    std::vector<ControllerInterface*> controllers;
    controllers.push_back(new Ackerman());

    //Setting tolerance to reach goals
    controllers.at(0)->setTolerance(0.5);

    //Goals
    pfms::geometry_msgs::Point goal0{8,-1};
    pfms::geometry_msgs::Point goal1{-5,-2};

    std::vector<pfms::geometry_msgs::Point*> goals;
    goals.push_back(&goal0);
    goals.push_back(&goal1);

    //Below now is an analysis of the platforms for each goal
    //Ackerman can reach goal 0
    {
        bool reachable = controllers.at(0)->setGoal(goal0);
        double dist = controllers.at(0)->distanceToGoal();
        double t = controllers.at(0)->timeToGoal();
        std::cout << "Ackerman: can reach goal " <<
                             dist << "[m] " << t << "[s]" << std::endl;
        ASSERT_TRUE(reachable);
    }

    //The Ackerman can reach goal 1
    {
        bool reachable = controllers.at(0)->setGoal(goal1);
        double dist = controllers.at(0)->distanceToGoal();
        double t = controllers.at(0)->timeToGoal();
        std::cout << "Ackerman: can reach goal " <<
                             dist << "[m] " << t << "[s]" << std::endl;
        ASSERT_TRUE(reachable);
    }


    //////////////////////////////////////////////////////////////////////////////////
    // Let's now check missions
    Mission mission(controllers);
    mission.setMissionObjective(mission::Objective::DISTANCE);
    mission.setGoals(goals);

    {
        std::vector<unsigned int> assignment =  mission.getPlatformGoalAssociation();

        for(unsigned int i=0;i<assignment.size();i++){
            std::cout << i << " : " << assignment.at(i) << std::endl;
        }

        ASSERT_EQ(assignment.size(),goals.size());//Need to have assigned a platform for each goal
        ASSERT_EQ(assignment.at(0),0); // Platform 0 should be going to goal 0
        ASSERT_EQ(assignment.at(1),0); // Platform 0 should be going to goal 1
    }


    bool missionOK = mission.runMission();

    // The mission should succseed
    EXPECT_TRUE(missionOK);

    // The Ackerman should not be within 0.5m of goal 0
    {
        pfms::nav_msgs::Odometry odo = controllers.at(0)->getOdometry();
        double distance = pow ( pow(odo.position.x-goal0.x,2) + pow(odo.position.y-goal0.y,2),0.5);
        std::cout << "Distance to goal:" << distance << std::endl;
        ASSERT_FALSE(distance < 0.5);
    }

    // The Ackerman should be within 0.5m of goal 1
    {
        pfms::nav_msgs::Odometry odo = controllers.at(0)->getOdometry();
        double distance = pow ( pow(odo.position.x-goal1.x,2) + pow(odo.position.y-goal1.y,2),0.5);
        std::cout << "Distance to goal:" << distance << std::endl;
        ASSERT_NEAR(distance,0,0.5);
    }


}




int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
