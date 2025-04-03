#include "gtest/gtest.h"
#include <iostream>
#include <vector>
#include <algorithm>
#include <cmath>

//Student defined libraries
#include "quadcopter.h"

// Some helper header for assembling messages and testing
#include "pfms_types.h"
#include "test_helper.h"
#include "linkcommand.h"

using namespace std;
using namespace pfms::nav_msgs;


TEST(Quadcopter,checkOriginToDestination) {

    //The below could be used to initiate class that can teleport the platform to any position
    //LinkCommand* linkCommand = new LinkCommand;

    std::vector<ControllerInterface*> controllers;
    controllers.push_back(new Quadcopter());
    controllers.front()->setTolerance(0.5);

    pfms::nav_msgs::Odometry estimatedGoalPose;
    double dist, t;
    bool reachable;
    double toleranceDist=0.5;
    double toleranceYaw=M_PI/32;


    {
        pfms::nav_msgs::Odometry odo{1,0,1.99886,2.0,0.000175213,0,0};
        // If you wish to see your platform in any of the odo pose simply uncomment
        //     linkCommand->writeCommand(odo);
        pfms::geometry_msgs::Point pt{10,-2};
        reachable = controllers.at(0)->checkOriginToDestination(odo,pt,dist,t,estimatedGoalPose);
        ASSERT_TRUE(reachable);
        ASSERT_NEAR(estimatedGoalPose.position.x,pt.x,toleranceDist);
        ASSERT_NEAR(estimatedGoalPose.position.y,pt.y,toleranceDist);
        ASSERT_NEAR(estimatedGoalPose.yaw,odo.yaw, toleranceYaw);
    }

    {
        pfms::nav_msgs::Odometry odo{2,9.70361,-1.71193,2.0,-0.753051,0,0};
        pfms::geometry_msgs::Point pt{10,6};
        reachable = controllers.at(0)->checkOriginToDestination(odo,pt,dist,t,estimatedGoalPose);
        ASSERT_TRUE(reachable);
        ASSERT_NEAR(estimatedGoalPose.position.x,pt.x,toleranceDist);
        ASSERT_NEAR(estimatedGoalPose.position.y,pt.y,toleranceDist);
        ASSERT_NEAR(estimatedGoalPose.yaw,odo.yaw, toleranceYaw);
    }

    {
        pfms::nav_msgs::Odometry odo{3,10.3968,6.35645,2.0,-2.50378,0,0};
        pfms::geometry_msgs::Point pt{0,-2};
        reachable = controllers.at(0)->checkOriginToDestination(odo,pt,dist,t,estimatedGoalPose);
        ASSERT_TRUE(reachable);
        ASSERT_NEAR(estimatedGoalPose.position.x,pt.x,toleranceDist);
        ASSERT_NEAR(estimatedGoalPose.position.y,pt.y,toleranceDist);
        ASSERT_NEAR(estimatedGoalPose.yaw,odo.yaw, toleranceYaw);
    }

    {
        pfms::nav_msgs::Odometry odo{4,0.375999,-1.72774,2.0,-2.51173,0,0};
        pfms::geometry_msgs::Point pt{-3,-20};
        reachable = controllers.at(0)->checkOriginToDestination(odo,pt,dist,t,estimatedGoalPose);

        ASSERT_TRUE(reachable);
        ASSERT_NEAR(estimatedGoalPose.position.x,pt.x,toleranceDist);
        ASSERT_NEAR(estimatedGoalPose.position.y,pt.y,toleranceDist);
        ASSERT_NEAR(estimatedGoalPose.yaw,odo.yaw, toleranceYaw);
    }

    {
        pfms::nav_msgs::Odometry odo{5,-3.24623,-19.6444,2.0,-0.98752,0,0};
        pfms::geometry_msgs::Point pt{-11,-20};
        reachable = controllers.at(0)->checkOriginToDestination(odo,pt,dist,t,estimatedGoalPose);
        ASSERT_TRUE(reachable);
        ASSERT_NEAR(estimatedGoalPose.position.x,pt.x,toleranceDist);
        ASSERT_NEAR(estimatedGoalPose.position.y,pt.y,toleranceDist);
        ASSERT_NEAR(estimatedGoalPose.yaw,odo.yaw, toleranceYaw);
    }

    {
        pfms::nav_msgs::Odometry odo{6,-11.3292,-20.4338,2.0,1.15282,0,0};
        pfms::geometry_msgs::Point pt{-10,40};
        reachable = controllers.at(0)->checkOriginToDestination(odo,pt,dist,t,estimatedGoalPose);
        ASSERT_TRUE(reachable);
        ASSERT_NEAR(estimatedGoalPose.position.x,pt.x,toleranceDist);
        ASSERT_NEAR(estimatedGoalPose.position.y,pt.y,toleranceDist);
        ASSERT_NEAR(estimatedGoalPose.yaw,odo.yaw, toleranceYaw);
    }

    {
        pfms::nav_msgs::Odometry odo{7,-9.78418,39.6225,2.0,2.08226,0,0};
        pfms::geometry_msgs::Point pt{-18,40};
        reachable = controllers.at(0)->checkOriginToDestination(odo,pt,dist,t,estimatedGoalPose);
        ASSERT_TRUE(reachable);
        ASSERT_NEAR(estimatedGoalPose.position.x,pt.x,toleranceDist);
        ASSERT_NEAR(estimatedGoalPose.position.y,pt.y,toleranceDist);
        ASSERT_NEAR(estimatedGoalPose.yaw,odo.yaw, toleranceYaw);
    }

    {
        pfms::nav_msgs::Odometry odo{8,-17.7172,40.4074,2.0,-2.23692,0,0};
        pfms::geometry_msgs::Point pt{-20,9};
        reachable = controllers.at(0)->checkOriginToDestination(odo,pt,dist,t,estimatedGoalPose);
        ASSERT_TRUE(reachable);
        ASSERT_NEAR(estimatedGoalPose.position.x,pt.x,toleranceDist);
        ASSERT_NEAR(estimatedGoalPose.position.y,pt.y,toleranceDist);
        ASSERT_NEAR(estimatedGoalPose.yaw,odo.yaw, toleranceYaw);
    }

    {
        pfms::nav_msgs::Odometry odo{9,-20.2001,9.34087,2.0,-1.05205,0,0};
        pfms::geometry_msgs::Point pt{-15,5};
        reachable = controllers.at(0)->checkOriginToDestination(odo,pt,dist,t,estimatedGoalPose);
        ASSERT_TRUE(reachable);
        ASSERT_NEAR(estimatedGoalPose.position.x,pt.x,toleranceDist);
        ASSERT_NEAR(estimatedGoalPose.position.y,pt.y,toleranceDist);
        ASSERT_NEAR(estimatedGoalPose.yaw,odo.yaw, toleranceYaw);
    }

    {
        pfms::nav_msgs::Odometry odo{10,-14.9721,4.89091,2.0,-0.376406,0,0};
        pfms::geometry_msgs::Point pt{0,2};
        reachable = controllers.at(0)->checkOriginToDestination(odo,pt,dist,t,estimatedGoalPose);
        ASSERT_TRUE(reachable);
        ASSERT_NEAR(estimatedGoalPose.position.x,pt.x,toleranceDist);
        ASSERT_NEAR(estimatedGoalPose.position.y,pt.y,toleranceDist);
        ASSERT_NEAR(estimatedGoalPose.yaw,odo.yaw, toleranceYaw);
    }

    {
        pfms::nav_msgs::Odometry odo{11,-0.448138,1.99516,2.0,0.00127136,0,0};
        pfms::geometry_msgs::Point pt{10,-2};
        reachable = controllers.at(0)->checkOriginToDestination(odo,pt,dist,t,estimatedGoalPose);
        ASSERT_TRUE(reachable);
        ASSERT_NEAR(estimatedGoalPose.position.x,pt.x,toleranceDist);
        ASSERT_NEAR(estimatedGoalPose.position.y,pt.y,toleranceDist);
        ASSERT_NEAR(estimatedGoalPose.yaw,odo.yaw, toleranceYaw);
    }

    {
        pfms::nav_msgs::Odometry odo{12,9.70908,-1.72546,2.0,-0.738129,0,0};
        pfms::geometry_msgs::Point pt{10,6};
        reachable = controllers.at(0)->checkOriginToDestination(odo,pt,dist,t,estimatedGoalPose);
        ASSERT_TRUE(reachable);
        ASSERT_NEAR(estimatedGoalPose.position.x,pt.x,toleranceDist);
        ASSERT_NEAR(estimatedGoalPose.position.y,pt.y,toleranceDist);
        ASSERT_NEAR(estimatedGoalPose.yaw,odo.yaw, toleranceYaw);
    }

    {
        pfms::nav_msgs::Odometry odo{13,10.2939,6.31255,2.0,-2.50665,0,0};
        pfms::geometry_msgs::Point pt{0,-2};
        reachable = controllers.at(0)->checkOriginToDestination(odo,pt,dist,t,estimatedGoalPose);
        ASSERT_TRUE(reachable);
        ASSERT_NEAR(estimatedGoalPose.position.x,pt.x,toleranceDist);
        ASSERT_NEAR(estimatedGoalPose.position.y,pt.y,toleranceDist);
        ASSERT_NEAR(estimatedGoalPose.yaw,odo.yaw, toleranceYaw);
    }

    {
        pfms::nav_msgs::Odometry odo{14,0.39936,-1.74734,2.0,-2.57073,0,0};
        pfms::geometry_msgs::Point pt{-3,-20};
        reachable = controllers.at(0)->checkOriginToDestination(odo,pt,dist,t,estimatedGoalPose);
        ASSERT_TRUE(reachable);
        ASSERT_NEAR(estimatedGoalPose.position.x,pt.x,toleranceDist);
        ASSERT_NEAR(estimatedGoalPose.position.y,pt.y,toleranceDist);
        ASSERT_NEAR(estimatedGoalPose.yaw,odo.yaw, toleranceYaw);
    }

    {
        pfms::nav_msgs::Odometry odo{15,-3.2195,-19.7217,2.0,-0.922661,0,0};
        pfms::geometry_msgs::Point pt{-11,-20};
        reachable = controllers.at(0)->checkOriginToDestination(odo,pt,dist,t,estimatedGoalPose);
        ASSERT_TRUE(reachable);
        ASSERT_NEAR(estimatedGoalPose.position.x,pt.x,toleranceDist);
        ASSERT_NEAR(estimatedGoalPose.position.y,pt.y,toleranceDist);
        ASSERT_NEAR(estimatedGoalPose.yaw,odo.yaw, toleranceYaw);
    }

    {
        pfms::nav_msgs::Odometry odo{16,-11.3283,-20.4052,2.0,1.07015,0,0};
        pfms::geometry_msgs::Point pt{-10,40};
        reachable = controllers.at(0)->checkOriginToDestination(odo,pt,dist,t,estimatedGoalPose);
        ASSERT_TRUE(reachable);
        ASSERT_NEAR(estimatedGoalPose.position.x,pt.x,toleranceDist);
        ASSERT_NEAR(estimatedGoalPose.position.y,pt.y,toleranceDist);
        ASSERT_NEAR(estimatedGoalPose.yaw,odo.yaw, toleranceYaw);
    }

    {
        pfms::nav_msgs::Odometry odo{18,-17.7293,40.3356,-2.2978,0,0};
        pfms::geometry_msgs::Point pt{-20,9};
        reachable = controllers.at(0)->checkOriginToDestination(odo,pt,dist,t,estimatedGoalPose);
        ASSERT_TRUE(reachable);
        ASSERT_NEAR(estimatedGoalPose.position.x,pt.x,toleranceDist);
        ASSERT_NEAR(estimatedGoalPose.position.y,pt.y,toleranceDist);
        ASSERT_NEAR(estimatedGoalPose.yaw,odo.yaw, toleranceYaw);
    }

    {
        pfms::nav_msgs::Odometry odo{19,-20.2405,9.36119,-0.996773,0,0};
        pfms::geometry_msgs::Point pt{-15,5};
        reachable = controllers.at(0)->checkOriginToDestination(odo,pt,dist,t,estimatedGoalPose);
        ASSERT_TRUE(reachable);
        ASSERT_NEAR(estimatedGoalPose.position.x,pt.x,toleranceDist);
        ASSERT_NEAR(estimatedGoalPose.position.y,pt.y,toleranceDist);
        ASSERT_NEAR(estimatedGoalPose.yaw,odo.yaw, toleranceYaw);
    }

    {
        pfms::nav_msgs::Odometry odo{20,-15.0173,4.93128,-0.427219,0,0};
        pfms::geometry_msgs::Point pt{0,2};
        reachable = controllers.at(0)->checkOriginToDestination(odo,pt,dist,t,estimatedGoalPose);
        ASSERT_TRUE(reachable);
        ASSERT_NEAR(estimatedGoalPose.position.x,pt.x,toleranceDist);
        ASSERT_NEAR(estimatedGoalPose.position.y,pt.y,toleranceDist);
        ASSERT_NEAR(estimatedGoalPose.yaw,odo.yaw, toleranceYaw);
    }

}


int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
