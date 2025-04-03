#include "controller.h"
#include <cmath>


///////////////////////////////////////////////////////////////
//! @todo
//! TASK 1 - Initialisation
//!
//! Is there anything we need to initialise in the Constructor?

Controller::Controller() :
    odo_{0,0,0,0,0,0},
    distance_travelled_(0),
    time_travelled_(0),
    cmd_pipe_seq_(0)

    
{
    // We open up the pipes here in the constructor, so we can OPEN them once ONLY
    // and close them in desctructor.
    // As they are part of base class, we can not craete them via initialiser list
    pipesPtr_ = new Pipes();

    //We set the internal variables of time/distance for goal to zero
    goal_.time=0;
    goal_.distance=0;
};


bool Controller::setGoal(pfms::geometry_msgs::Point goal) {
  goal_.location = goal;
  return calcNewGoal();
}

bool Controller::setTolerance(double tolerance) {
  tolerance_ = tolerance;
  return true;
}

double Controller::distanceToGoal(void) {
    return goal_.distance;
}
double Controller::timeToGoal(void) {
    return goal_.time;
}
double Controller::distanceTravelled(void) {
    return distance_travelled_;
}
double Controller::timeInMotion(void) {
    return time_travelled_;
}

bool Controller::goalReached() {
    double dx = goal_.location.x - odo_.position.x;
    double dy = goal_.location.y - odo_.position.y;

    return (pow(pow(dx,2)+pow(dy,2),0.5) < tolerance_);
}


pfms::nav_msgs::Odometry Controller::getOdometry(void){
    pipesPtr_->read(odo_,type_);
    return odo_;
}

pfms::PlatformType Controller::getPlatformType(void){
    return type_;
}
