#include "sample.h"

/**
 * This sample code is provided to illustrate
 * - Subscribing to standard topics (Odometry, Laser)
 * - Publishing visualisation markers
 * - The need to exchange data between seperate thread of execution and callbacks
 */

using std::cout;
using std::endl;


PfmsSample::PfmsSample(ros::NodeHandle nh)
    : nh_(nh), marker_counter_(0)
{
    //Subscribing to odometry
    sub1_ = nh_.subscribe("robot_0/odom", 1000, &PfmsSample::odomCallback,this);
    //Subscribing to laser
    sub2_ = nh_.subscribe("robot_0/base_scan_1", 10, &PfmsSample::laserCallback,this);

    //Publishing an marker array  ... for visualisation
    viz_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("visualization_marker",3,false);

    // Below is how to get parameters from command line, on command line they need to be _param:=value
    // For example _example:=0.1
    // ROS will obtain the configuration from command line, or assign a default value 0.1
    ros::NodeHandle pn("~");
    double example;
    pn.param<double>("example", example, 0.1);
    ROS_INFO_STREAM("param example:" << example);
}

PfmsSample::~PfmsSample()
{

}



// A callback for odometry
void PfmsSample::odomCallback(const nav_msgs::OdometryConstPtr& msg)
{
    /**
     * @todo - Ex 1: Obtain a pose (x,y yaw) from nav_msgs/Odometry
     *

     * - On command line type 'rosmsg show nav_msgs/Odometry'
     * - The position and orientation are in two seperate parts of the message
     * - The orinetation is provided as a quaternion
     * - Which angle to we need?
     * - Ros has a 'tf' library with a helper function to get yaw from the quaternion
     * - http://docs.ros.org/diamondback/api/tf/html/c++/namespacetf.html
     * - Consider: Do we have nav_msgs::Odometry or q pointer to nav_msgs::Odometry ?
     * - Where is time of this message stored
     */


    geometry_msgs::Pose pose = msg->pose.pose;

// We can use 2 different verbosty levels, and swicth then ON/OFF dynamically
//  ROS_INFO_STREAM("x: " << msg->pose.pose.position.x
//              << ",  y: " << msg->pose.pose.position.y
//              << ",  yaw: "<< tf::getYaw(msg->pose.pose.orientation));
//  ROS_DEBUG_STREAM("x: " << msg->pose.pose.position.x
//                << ",  y: " << msg->pose.pose.position.y
//                << ",  yaw: "<< tf::getYaw(msg->pose.pose.orientation));

    std::unique_lock<std::mutex> lck1 (robotPoseMtx_);
    robotPose_ = pose; // We store a copy of the pose in robotPose_
}



void PfmsSample::laserCallback(const sensor_msgs::LaserScanConstPtr& msg)
{

  /**
   * @todo - Ex 1 : Find the closest point {x,y} to the robot using sensor_msgs::LaserScan
   *
   * On command line type 'rosmsg show sensor_msgs/LaserScan'
   * What are we provided in this message?
   * Do we have the information in this message to find the closest point?
   * What part of the message do we need to iterate over?
   * How do we convert from range data to {x,y} [this is known as polar to cartesian](https://www.mathsisfun.com/polar-cartesian-coordinates.html)
   * Where is time of this message stored?
   * Is the closest point identified the same as the one you see as closest on the stage simulator? Why is this the case?
   */

    std::unique_lock<std::mutex> lck(laserDataMtx_);
    laserData_ = *msg; // We store a copy of the LaserScan in laserData_
    lck.unlock();

    // LaserProcessing laserProcessing(laserData_);
    // geometry_msgs::Point pt = laserProcessing.closestPoint();

    // ROS_INFO_STREAM(msg->header.stamp << " x:" << pt.x << " y:" << pt.y);
}

void PfmsSample::seperateThread() {
   /**
    * The below loop runs until ros is shutdown, to ensure this thread does not remain
    * a zombie thread
    *
    */

    //Our thread will start momenteraly
    //So it is possible it may start before the callbacks.
    //If you do not want that to start you might want to wait for both callbacks.
    //How would you do this? 

    // The below gets the current Ros Time is needed
    //ros::Time timeSample = ros::Time::now();

    //! What does this rate limiter do?
    ros::Rate rate_limiter(1.0);

    while (ros::ok()) {

      /**
       * @todo Find and mark the closest point in global coordinates [x,y]
       * using the robot pose and closest point in laser data
       *
       * - We need to combine the information from pose and laser into this thread
       * - We use the visualisation code for marker array
       */

      //Let's get pose
      std::unique_lock<std::mutex> lck1 (robotPoseMtx_);
      std::unique_lock<std::mutex> lck2 (laserDataMtx_);

      //Let's get the yaw:
      double r_yaw =  tf::getYaw(robotPose_.orientation);

      //Let's use our LaserProcessing code to get the closest point to robot
      LaserProcessing laserProcessing(laserData_);
      geometry_msgs::Point pt = laserProcessing.closestPoint();

      // Computing the global position, you should consider putting this in a function
      // so you can unit test it (like quiz 3)
      double x_g = (pt.x * cos(r_yaw) - pt.y * sin(r_yaw)) + robotPose_.position.x;
      double y_g = (pt.x * sin(r_yaw) + pt.y * cos(r_yaw)) + robotPose_.position.y;

      lck2.unlock();
      lck1.unlock();

      ROS_INFO_STREAM("x:" << x_g << " y:" << y_g);

      //We create a marker and push it back to marker array instead
      visualization_msgs::Marker marker = createMarker(x_g,y_g,robotPose_.position.z);
      visualization_msgs::MarkerArray marker_array;
      marker_array.markers.push_back(marker);

      //We publish the marker array
      viz_pub_.publish(marker_array);

      //We have a rate timer, this sleep here is needed to ensure it stops and sleeps 
      //it will do it for the exact amount of time needed to run at 1Hz
      rate_limiter.sleep();
    }
}

visualization_msgs::Marker PfmsSample::createMarker(double x, double y,double z){
  //! Here is an example of creating a marker 
  //!
  //!
  visualization_msgs::Marker marker;

  //We need to set the frame
  // Set the frame ID and time stamp.
  marker.header.frame_id = "world";

  //single_marker_person.header.stamp = ros::Time();
  marker.header.stamp = ros::Time::now();


  //We set lifetime (it will dissapear in this many seconds)
  marker.lifetime = ros::Duration(5.0);
  // Set the namespace and id for this marker.  This serves to create a unique ID
  // Any marker sent with the same namespace and id will overwrite the old one
  marker.ns = "test";
  
  marker.id = marker_counter_++; 

  // The marker type, we use a cylinder in this example
  marker.type = visualization_msgs::Marker::CYLINDER;

  // Set the marker action.  Options are ADD and DELETE (we ADD it to the screen)
  marker.action = visualization_msgs::Marker::ADD;

  marker.pose.position.x = x;
  marker.pose.position.y = y;
  marker.pose.position.z = z;

  //Orientation, we are not going to orientate it
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;


  // Set the scale of the marker -- 0.5x0.5x0.5 here means 0.5m side
  marker.scale.x = 0.5;
  marker.scale.y = 0.5;
  marker.scale.z = 0.5;

  //Alpha is stransparency (50% transparent)
  marker.color.a = 0.5f;

  //Colour is r,g,b where each channel of colour is 0-1. Bellow will make it orange
  marker.color.r = 1.0;
  marker.color.g = static_cast<float>(177.0/255.0);
  marker.color.b = 0.0;

  return marker;
}
