#include "sample.h"
#include <iostream>
#include <cmath>
#include <thread>
#include <chrono>
#include <time.h>

#include "std_msgs/Float64.h"
#include "std_msgs/Empty.h"
#include "geometry_msgs/Twist.h"

#include "tf/transform_datatypes.h" //To use getYaw function from the quaternion of orientation

using std::cout;
using std::endl;

Sample::Sample(ros::NodeHandle nh) :
  nh_(nh),
  laserProcessingPtr_(nullptr)
{


    //Subscribing to odometry UGV
    sub1_ = nh_.subscribe("ugv_odom", 1000, &Sample::odomCallback,this);

    //Subscribing to odometry UAV
    //sub1_ = nh_.subscribe("uav_odom", 1000, &Sample::odomCallback,this);


    //Publishing markers
    pub_ = nh_.advertise<visualization_msgs::MarkerArray>("visualization_marker",3,false);

    //Allowing an incoming service on /request_goal (you need to change name depending on project)
    service_ = nh_.advertiseService("request_goals", &Sample::request,this);


};

// We delete anything that needs removing here specifically
Sample::~Sample(){

    if(laserProcessingPtr_ != nullptr){
        delete laserProcessingPtr_;
    }
}


// A callback for odometry
void Sample::odomCallback(const nav_msgs::OdometryConstPtr &msg)
{
    
}



void Sample::seperateThread() {
   /**
    * The below loop runs until ros is shutdown
    */

    //! THINK : What rate shouls we run this at? What does this limiter do?
    ros::Rate rate_limiter(5.0);
    while (ros::ok()) {

        rate_limiter.sleep();

    }
}



bool Sample::request(std_srvs::SetBool::Request  &req,
             std_srvs::SetBool::Response &res)
{
    //When an incoming call arrives, we can respond to it here
    //Check what is in the service via  rossrv info project_setup/RequestGoal

    ROS_INFO_STREAM("Requested:" << req.data);

    return true; //We return true to indicate the service call sucseeded (your responce should indicate a value)
}