#include <cmath>
#include <iostream>

#include "quadcopter.h"
#include "pipes.h"


Quadcopter::Quadcopter(){
    platformType_ = pfms::PlatformType::QUADCOPTER;
    // Initialize some variables 
    // platformType_ = pfms::PlatformType::ACKERMAN;
    // distanceTravelled_ = 0;
    // timeInMotion_ = 0;
}


bool Controller::setGoals(std::vector<pfms::geometry_msgs::Point> goals){
    goals_ = goals;

    pfms::nav_msgs::Odometry estimatedGoalPose;

    // Update these variables for quadcopterReachGoal test only
    // Not the main purpose of checkOriginToDestination
    Controller::getOdometry();
    checkOriginToDestination(odo_, goals_.at(0), distanceToGoal_, timeToGoal_, estimatedGoalPose);

    return true;
}

bool Quadcopter::checkOriginToDestination(pfms::nav_msgs::Odometry origin,
                                        pfms::geometry_msgs::Point goal,
                                        double& distance,
                                        double& time,
                                        pfms::nav_msgs::Odometry& estimatedGoalPose){
    

    estimatedGoalPose.position.x = goal.x;
    estimatedGoalPose.position.y = goal.y;
    estimatedGoalPose.yaw = origin.yaw;

    distance = euclideanDistance(origin.position.x, origin.position.y, goal.x, goal.y);

    // ???
    time = distance / 0.4;
    
    return true;
    }

double Quadcopter::distanceToGoal(void){
    return distanceToGoal_;
}

double Quadcopter::timeToGoal(void){
    return timeToGoal_;
}