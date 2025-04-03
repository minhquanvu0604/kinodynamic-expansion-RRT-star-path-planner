#include "controller.h"
#include "pipes.h"

Controller::Controller(){
    // Create the pipe 
    pipesPtr = new Pipes();
}

// 4
pfms::PlatformType Controller::getPlatformType(void){
    return platformType_;
}

// 5
double Controller::distanceToGoal(void){
    return distance_;
}

// 6
double Controller::timeToGoal(void){
    return time_;
}

// 7
bool Controller::setTolerance(double tolerance){
    tolerance_ = tolerance;
    return true;
}

// 10
pfms::nav_msgs::Odometry Controller::getOdometry(void){
    return odo_;
}

