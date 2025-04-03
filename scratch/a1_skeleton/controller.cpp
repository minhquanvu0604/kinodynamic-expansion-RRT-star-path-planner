#include "controller.h"
#include "pipes.h"

Controller::Controller(){
    
    // Create the pipe 
    pipesPtr = new Pipes();
}

Controller::~Controller(){
    delete pipesPtr;
}

pfms::PlatformType Controller::getPlatformType(void){
    return platformType_;
}

double Controller::distanceToGoal(void){
    return distanceToGoal_;
}

double Controller::timeToGoal(void){
    return timeToGoal_;
}

bool Controller::setTolerance(double tolerance){
    if (tolerance >= 0) {
        tolerance_ = tolerance;
        return true;
    }
    else {
        tolerance_ = std::abs(tolerance);
        return false;
    }
}

double Controller::timeInMotion(void){
    return timeInMotion_;
}


// HELPER FUNCTIONS

double Controller::euclideanDistance(double x1, double y1, double x2, double y2) {
    return std::sqrt(std::pow(x2 - x1, 2) + std::pow(y2 - y1, 2));
}

double Controller::normalizeAngle(double angle)
{
    angle = std::fmod(angle + M_PI, 2 * M_PI);
    if (angle < 0) {
        angle += 2 * M_PI;
    }
    return angle - M_PI;
}
