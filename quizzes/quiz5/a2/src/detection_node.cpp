
#include "detection_node.h"
#include "rangeprocessing.h"


/**
 * This node shows some connections and publishing images
 */


Detection::Detection(ros::NodeHandle nh)
    : nh_(nh)
{
    //Subscribing to odometry (Do we need to change thge topic???)
    sub1_ = nh_.subscribe("/ugv_odom", 1000, &Detection::odomCallback,this);

    //Subscribing to occupnacy grid
    sub2_ = nh_.subscribe("drone/sonar/range", 1, &Detection::rangeCallback,this);

}

Detection::~Detection()
{

}


void Detection::odomCallback(const nav_msgs::OdometryConstPtr& msg)
{
    //! REMEBER: on command line you can view entier msg as
    //! rosmsg show nav_msgs/Odometry
    std::unique_lock<std::mutex> lck (poseDataBuffer_.mtx);
    poseDataBuffer_.pose = msg->pose.pose;
}



void Detection::rangeCallback(const sensor_msgs::RangeConstPtr &msg)
{

  std::unique_lock<std::mutex> lck (rangeBuffer_.mtx);
  rangeBuffer_.range = *msg;

}


void Detection::seperateThread() {

   /**
    * The below loop runs until ros is shutdown, to ensure this thread does not remain
    * a zombie thread
    *
    */
   geometry_msgs::Point person;
   sensor_msgs::Range range;
   geometry_msgs::Pose pose;


    //! rate limiter runs code every 5 seconds
    ros::Rate rate_limiter(1.0/5.0);

    while (ros::ok()) {

        //! Get the Pose message
        poseDataBuffer_.mtx.lock();
        pose=poseDataBuffer_.pose;
        poseDataBuffer_.mtx.unlock();


        //! Get the OgMap message
        rangeBuffer_.mtx.lock();
        range= rangeBuffer_.range;
        rangeBuffer_.mtx.unlock();


//            OgmapProcessing ogmapProcessing(grid);
//            bool OK = ogmapProcessing.isLocationFree(reqPose.position);

//            if(OK){
//              ROS_INFO_STREAM("Cell is free");
//            }
//            else {
//              ROS_INFO_STREAM("Cell is NOT free");
//            }

        rate_limiter.sleep();
    }
}

