#ifndef SAMPLE_H
#define SAMPLE_H

#include "ros/ros.h"
#include <atomic>
#include <mutex>

//Keep only the headers needed
#include "visualization_msgs/MarkerArray.h"
#include "std_srvs/SetBool.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Range.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/PoseArray.h"

//We include header of anotehr class we are developing
#include "laserprocessing.h"

//! Sample
class Sample
{
public:
  //Default constructor - should set all sensor attributes to a default value
  Sample(ros::NodeHandle nh);

  ~Sample();


  /*! @brief seperate thread.
  *
  *  The main processing thread that will run continously and utilise the data
  *  When data needs to be combined then running a thread seperate to callback will gurantee data is processed
  */
  void seperateThread();

  /*! @brief request service callback
   *
   *  @param req The request
   *  @param res The responce
   *
   *  @return bool - Will return true to indicate the request sucseeded
   */
  bool request(std_srvs::SetBool::Request  &req,
               std_srvs::SetBool::Response &res);

private:

  /*! @brief Odometry Callback
   *
   *  @param nav_msgs::OdometryConstPtr - The odometry message
   *  @note This function and the declaration are ROS specific
   */
    void odomCallback(const nav_msgs::OdometryConstPtr& msg);
 
private:

  ros::NodeHandle nh_;//Node handle for communication

  ros::Publisher pub_;//! Visualisation Marker publisher

  ros::Subscriber sub1_;  // Few subscribers
  ros::ServiceServer service_; // Incoming service

  LaserProcessing* laserProcessingPtr_; //! Pointer to Laser Object

};

#endif // SAMPLE_H
