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
//! TASK 1 - Initialisation
//!
//! What do we need to subscribe to and publish?

Quadcopter::Quadcopter(ros::NodeHandle nh) :
    TARGET_SPEED(0.4),liftoff_(false),
    TARGET_HEIGHT_TOLERANCE(0.2)
{
   tolerance_=0.5;//We set tolerance to be default of 0.5
   pubCmdVel_  = nh_.advertise<geometry_msgs::Twist>("drone/cmd_vel",3,false);  
   pubTakeOff_ = nh_.advertise<std_msgs::Empty>("drone/takeoff",3,false); 

   //Allowing an incoming service on /reach_goal (you need to change name depending on project)
   service_ = nh_.advertiseService("reach_goal", &Quadcopter::request,this);    

};

Quadcopter::~Quadcopter(){
   
}


bool Quadcopter::request(std_srvs::SetBool::Request  &req,
             std_srvs::SetBool::Response &res)
{
    //When an incoming call arrives, we can respond to it here
    //Check what is in the service via rossrv info std_srvs/SetBool
    ROS_INFO_STREAM("Requested:" << req.data);

    //We can reply in the two field of the return value
    res.success = true;
    res.message = "Message";

    return true; //We return true to indicate the service call sucseeded (your responce should indicate a value)
}

void Quadcopter::sendCmd(double turn_l_r, double move_l_r, double move_u_d, double move_f_b) {

    if(!liftoff_){
        std_msgs::Empty msg;
        pubTakeOff_.publish(msg);
    }

    geometry_msgs::Twist msg;
    msg.linear.x= move_f_b;
    msg.linear.y= move_l_r;
    msg.linear.z= move_u_d;
    msg.angular.z = turn_l_r;
    pubCmdVel_.publish(msg);
}

void Quadcopter::reachGoal(void) {

    ros::Rate rate_limiter(20.0);

    while (ros::ok()) {    


        if(!goalSet_){
            //it will do it for the exact amount of time needed to run at 1Hz
            rate_limiter.sleep();
            continue; //This will make the code return back to beginig of while loop
        };

        calcNewGoal(); // account for any drift between setGoal call and now, by getting odo and angle to drive in

        nav_msgs::Odometry odo = getOdometry();

        // Get relative target angle
        double theta = tf::getYaw(odo.pose.pose.orientation) - target_angle_;

        // Move at `speed` in target direction
        double dx = TARGET_SPEED * std::cos(theta);
        double dy = TARGET_SPEED * std::sin(theta);

        //What about the height?
        double dz=0;

        if(odo.pose.pose.position.z>(goal_.location.z+TARGET_HEIGHT_TOLERANCE)){
            dz=-0.05;
        }
        if(odo.pose.pose.position.z<(goal_.location.z+TARGET_HEIGHT_TOLERANCE)){
            dz=+0.05;
        }

        bool reached = goalReached();  

        if(reached){
            goalSet_=false;
            // Stop thq quadcopter immediately
            sendCmd(0, 0, 0, 0);
            ROS_INFO("Goal reached");
        }
        else{
            //Let's send command with these parameters
            sendCmd(0, -dy, dz, dx);
            //ROS_INFO_STREAM("sending: " << dx << " " << -dy << " " << dz);
        }
        
        //it will do it for the exact amount of time needed to run at 1Hz
        rate_limiter.sleep();

    }

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

    nav_msgs::Odometry est_final_pos;

    nav_msgs::Odometry odo = getOdometry();

    if (!checkOriginToDestination(odo, goal_.location, goal_.distance, goal_.time, est_final_pos))
        return false;

    // Calculate absolute travel angle required to reach goal
    double dx = goal_.location.x - odo.pose.pose.position.x;
    double dy = goal_.location.y - odo.pose.pose.position.y;
    target_angle_ = std::atan2(dy, dx);

    return true;
}


