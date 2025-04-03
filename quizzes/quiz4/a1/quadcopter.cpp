#include "quadcopter.h"

#include <iostream>
#include <cmath>
#include <thread>
#include <chrono>

#include <pfms_types.h>

///////////////////////////////////////////////////////////////
//! @todo
//! TASK 1 - Initialisation
//!
//! Is there anything we need to initialise in the Constructor?

Quadcopter::Quadcopter() :
    TARGET_SPEED{0.4}
{
  tolerance_=0.5;//We set tolerance to be default of 0.5
  type_ = pfms::QUADCOPTER;
};

// We delete the pipes here specifically, which forces them to close before the object is terminated
Quadcopter::~Quadcopter(){
    delete pipesPtr_;
}

bool Quadcopter::checkOriginToDestination(pfms::nav_msgs::Odometry origin, pfms::geometry_msgs::Point goal,
                              double& distance, double& time,
                              pfms::nav_msgs::Odometry& estimatedGoalPose) {

    // Use pythagorean theorem to get direct distance to goal
    double dx = goal.x - origin.position.x;
    double dy = goal.y - origin.position.y;

    distance = std::hypot(dx, dy);
    time = distance / TARGET_SPEED;

    // The estimated goal pose would be the goal, at the angle we had at the origin
    // as we are not rotating the platform, simple moving it left/right and fwd/backward
    estimatedGoalPose.position.x = goal.x;
    estimatedGoalPose.position.y = goal.y;
    estimatedGoalPose.yaw = origin.yaw;
    estimatedGoalPose.linear.x = 0;
    estimatedGoalPose.linear.y = 0;

    return true;
}
bool Quadcopter::calcNewGoal(void) {

    getOdometry();//This will update internal copy of odometry, as well as return value if needed.

    pfms::nav_msgs::Odometry est_final_pos;

    if (!checkOriginToDestination(odo_, goal_.location, goal_.distance, goal_.time, est_final_pos))
        return false;

    // Calculate absolute travel angle required to reach goal
    double dx = goal_.location.x * cos(odo_.yaw) + goal_.location.y * sin(odo_.yaw) - (odo_.position.x * cos(odo_.yaw) + odo_.position.y * sin(odo_.yaw));
    double dy = -goal_.location.x * sin(odo_.yaw) + goal_.location.y * cos(odo_.yaw) - (odo_.position.x * -sin(odo_.yaw) + odo_.position.y * cos(odo_.yaw));
    target_angle_ = std::atan2(dy, dx);

    return true;
}

void Quadcopter::sendCmd(double turn_l_r, double move_l_r, double move_u_d, double move_f_b) {
    pfms::commands::UAV cmd = {
        cmd_pipe_seq_++,
        turn_l_r,
        move_l_r,
        move_u_d,
        move_f_b,
    };
    pipesPtr_->send(cmd);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));//Small delay to ensure message sent
}

bool Quadcopter::reachGoal(void) {
    calcNewGoal(); // account for any drift between setGoal call and now, by getting odo and angle to drive in
    pfms::nav_msgs::Odometry prev_odo = odo_;
    auto start_time = std::chrono::system_clock::now();
    double estimated_time_to_reach_goal = goal_.time;

    pfms::nav_msgs::Odometry lastStepOdo = prev_odo;

    //Run below loop until we reach goal
    while (!goalReached()) {

        ///////////////////////////////////////////////////////////////
        //! @todo
        //! TASK 2 - Completion of task requires sending correct values
        //! in sendCmd
        //!
        //! We have access following variables
        //! odo_ - current odometry
        //! target_angle_ - the desired direction of travel
        //! AND the const TARGET_SPEED which is target speed (0.4ms/s)
        //!
        //! We need to compute
        //! dx and dy - the portion of the control in the two axis
        //! in below we just use 0.1 which will not work, it will fly at 45deg

        double dx = TARGET_SPEED * cos(target_angle_);
        double dy = TARGET_SPEED * sin(target_angle_);

        // Check the syntax of the command, for left/right and forward/backward
        sendCmd(0, dy, 0, dx);

        /////////////////////////////////////////////////////////////////


        // We check if we have not reached goal in close to anticipated time (2s margin)
        // if not reachGoal should be aborted
        double time_since_starting = timeLapsed(start_time);

       // std::cout << "[est/cur] time to goal [" << estimated_time_to_reach_goal << "/" << time_since_starting << "]" << std::endl;


        if(time_since_starting>(estimated_time_to_reach_goal+2.0)){
            // If we have not reached it in the designated time we abandon reaching goal
            // For TASK 3 and 4, consider if anything else needs updating
            return false;
        }

        calcNewGoal(); //get odometry and update target angle of control

        ///////////////////////////////////////////////////////////////
        //! @todo
        //! TASK 4 - Update distance travelled
        //!
        //! While the estimated distance at the begining is a rough guide
        //! As the platform moves it could travel more, especially if it
        //! was going against wind (and in case of Ackerman if it was drifting)
        //! Your better of incremeting distance travelled as you go along
        //!
        //! We have access to following variables
        //! odo_ - current position of platform
        //! prev_odo - the previous position
        //! distance_travelled_ - the distance travelled thus far

        // double speed = std::hypot(odo_.linear.x, odo_.linear.y);
        // std::cout << "SPEED: " << speed << std::endl;

        distance_travelled_ += (std::hypot((odo_.position.x - lastStepOdo.position.x), (odo_.position.y-lastStepOdo.position.y)));
        // std::cout << "Distance Travelled: " << distance_travelled_ << std::endl;

        lastStepOdo = odo_;
        //////////////////////////////////////////////////////////////////
    }

    // Stop thq quadcopter immediately
    sendCmd(0, 0, 0, 0);

    calcNewGoal(); //get odometry and update distance to goal

    ///////////////////////////////////////////////////////////////
    //! @todo
    //! TASK 3 - Update time travelled
    //!
    //! We have access following variables
    //! start_time - the time reaching the goal was started
    //! time_travelled_ - the time we have travelled thus far
    //!
    //! Look at how we compute wether to abort reachGoal
    //! as you could use the same function
    //! Also, consider, if we aborted the function reachGoal, should we
    //! update the time travelled?

    time_travelled_ += timeLapsed(start_time);

    //////////////////////////////////////////////////////////////////

    return true;
}


double Quadcopter::timeLapsed(std::chrono::time_point<std::chrono::system_clock> start_time){
    // Update time taken
    auto finish_time = std::chrono::system_clock::now();
    //std::chrono::seconds is integer for some reason, thus duration<double>
    auto time_taken = std::chrono::duration_cast<std::chrono::duration<double>>(finish_time - start_time);
    return time_taken.count();
}
