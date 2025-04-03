#include "detection_node.h"

Detection::Detection(ros::NodeHandle nh)
    : nh_(nh)
{
    sub1_ = nh_.subscribe("/orange/laser/scan", 10, &Detection::laserCallback,this);

    //Allowing an incoming service on /detect_road_centre
    service_ = nh_.advertiseService("/detect_closest", &Detection::detect,this);
}

Detection::~Detection()
{

}


void Detection::laserCallback(const sensor_msgs::LaserScanConstPtr& msg)
{
   /**
   * This callback store laser data to be used in service call
   */

    std::lock_guard<std::mutex> lck(laserData_.mtx);
    laserData_.scan = *msg;

}

bool Detection::detect(std_srvs::Empty::Request  &req,
                       std_srvs::Empty::Response &res)
{
    /**
   * The service is actually empty, does not take or pass arguments, but can be used to simply 
   * some code, or a function.
   */
    sensor_msgs::LaserScan scan;
    {
        std::lock_guard<std::mutex> lck(laserData_.mtx);
        scan = laserData_.scan;
    }


    LaserProcessing laserProcessing(scan);
    geometry_msgs::Point pt = laserProcessing.closestPoint();

    ROS_INFO_STREAM("Closest point x:" << pt.x << " y:" << pt.y);
    return true;
}

