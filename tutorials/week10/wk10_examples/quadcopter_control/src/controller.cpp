#include "controller.h"
#include <cmath>


/**
 * \brief Shared functionality/base class for platform controllers
 *
 */
Controller::Controller() :
    distance_travelled_(0),
    time_travelled_(0),
    cmd_pipe_seq_(0),
    goalSet_(false)
{
    // We open up the pipes here in the constructor, so we can OPEN them once ONLY
    //pipesPtr_ = new Pipes();
    // Now we create a node handle in derived class (as they have custom messages/topics)  
    // We still have one message we could potentialy use (odo)
    sub1_ = nh_.subscribe("/uav_odom/", 1000, &Controller::odoCallback,this);
    sub2_ = nh_.subscribe("/drone/goal", 1000, &Controller::setGoal,this);

    //We set the internal variables of time/distance for goal to zero
    goal_.time=0;
    goal_.distance=0;
};

//We would now have to sacrifice having a return value to have a setGoal
//At week 10 we do not know about services (which allow us to retrun value
//So to allow to set a goal via topic we forfit having areturn value for now
//At week 11 you can replace this with a service
//bool Controller::setGoal(geometry_msgs::Point goal) {
void Controller::setGoal(const geometry_msgs::Point::ConstPtr& msg){    
  goal_.location = *msg;
  goalSet_=true;
  //return calcNewGoal();
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
    double dx = goal_.location.x - odo_.pose.pose.position.x;
    double dy = goal_.location.y - odo_.pose.pose.position.y;
    double dz = goal_.location.z - odo_.pose.pose.position.z;

    return (pow(pow(dx,2)+pow(dy,2)+pow(dz,2),0.5) < tolerance_);
}

///////////////////////////////////////////////////////////////
//! @todo
//! TASK 3 - Callback 
//!
//! What do we do here?

void Controller::odoCallback(const nav_msgs::Odometry::ConstPtr& msg){
    odo_ = *msg;
}

//Do we need below ... maybe we can use it and impose a mutex
//To secure data?
nav_msgs::Odometry Controller::getOdometry(void){
    return odo_;
}

// pfms::PlatformType Controller::getPlatformType(void){
//     return type_;
// }
