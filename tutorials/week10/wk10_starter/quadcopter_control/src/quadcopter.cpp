#include "quadcopter.h"
#include <iostream>
#include <cmath>
#include <thread>
#include <chrono>
#include <time.h>

#define DEBUG 1

using std::cout;
using std::endl;

///////////////////////////////////////////////////////////////
//! @todo
//! TASK 3 - Initialisation
//!
//! What do we need to subscribe to and publish?

Quadcopter::Quadcopter(ros::NodeHandle nh) :
    TARGET_SPEED(0.4),liftoff_(false),TARGET_HEIGHT(2.0),TARGET_HEIGHT_TOLERANCE(0.2)
{
  // We open up the pipes here in the constructor, so we can OPEN them once ONLY
  // and close them in desctructor.
  // As they are part of base class, we can not craete them via initialiser list
  //type_ = pfms::PlatformType::QUADCOPTER; //Type is quadcopter
  tolerance_=0.5;//We set tolerance to be default of 0.5

  //What do we need to subscribe and what do we need to publish

//   //Subscribing to ???
//   sub1_ = nh_.subscribe("", 1000, &Quadcopter::callback,this);

//   //Publishing to
//   pub1_ = nh_.advertise<MSG_TYPE("topic_name",3,false);  


};

Quadcopter::~Quadcopter(){
   
}


bool Quadcopter::checkOriginToDestination(nav_msgs::Odometry origin, geometry_msgs::Point goal,
                              double& distance, double& time,
                              nav_msgs::Odometry& estimatedGoalPose) {

    // Use pythagorean theorem to get direct distance to goal
    double dx = goal.x - origin.pose.pose.position.x;
    double dy = goal.y - origin.pose.pose.position.y;

    distance = std::hypot(dx, dy);
    time = distance / TARGET_SPEED;

    // The estimated goal pose would be the goal, at the angle we had at the origin
    // as we are not rotating the platform, simple moving it left/right and fwd/backward
    estimatedGoalPose.pose.pose.position.x = goal.x;
    estimatedGoalPose.pose.pose.position.y = goal.y;
    //estimatedGoalPose.yaw = origin.yaw; - How do we deal with yaw to quaternion?

    return true;
}

bool Quadcopter::calcNewGoal(void) {

    getOdometry();//This will update internal copy of odometry, as well as return value if needed.

    nav_msgs::Odometry est_final_pos;

    if (!checkOriginToDestination(odo_, goal_.location, goal_.distance, goal_.time, est_final_pos))
        return false;

    // Calculate absolute travel angle required to reach goal
    double dx = goal_.location.x - odo_.pose.pose.position.x;
    double dy = goal_.location.y - odo_.pose.pose.position.y;
    target_angle_ = std::atan2(dy, dx);

    return true;
}

void Quadcopter::sendCmd(double turn_l_r, double move_l_r, double move_u_d, double move_f_b) {

    ///////////////////////////////////////////////////////////////
    //! @todo
    //! TASK 3 - Publishing values
    //!
    //! What do we need to publish here?

    // if(!liftoff_){
    //     pfms::PlatformStatus status = pfms::PlatformStatus::TAKEOFF;    
    //     pipesPtr_->send(status);
    //     std::this_thread::sleep_for(std::chrono::milliseconds(200));//Small delay to ensure message sent
    //     liftoff_=true;
    // }

    // pfms::commands::UAV cmd = {
    //     cmd_pipe_seq_++,
    //     turn_l_r,
    //     move_l_r,
    //     move_u_d,
    //     move_f_b,
    // };
    // pipesPtr_->send(cmd);

}

bool Quadcopter::reachGoal(void) {
    calcNewGoal(); // account for any drift between setGoal call and now, by getting odo and angle to drive in

    // Get relative target angle
    //double theta = odo_.yaw - target_angle_;
    double theta = 0;

    // Move at `speed` in target direction
    double dx = TARGET_SPEED * std::cos(theta);
    double dy = TARGET_SPEED * std::sin(theta);

    //What about the height?
    double dz=0;

    if(odo_.pose.pose.position.z>(TARGET_HEIGHT+TARGET_HEIGHT_TOLERANCE)){
        dz=-0.05;
    }
    if(odo_.pose.pose.position.z<(TARGET_HEIGHT+TARGET_HEIGHT_TOLERANCE)){
        dz=+0.05;
    }

    bool reached = goalReached();

    if(reached){
        // Stop thq quadcopter immediately
        sendCmd(0, 0, 0, 0);
    }
    else{
        //Let's send command with these parameters
        sendCmd(0, -dy, dz, dx);
    }

    return reached;
}
