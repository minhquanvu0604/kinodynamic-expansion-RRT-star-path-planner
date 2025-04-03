#include <sstream>
#include <iostream>
#include <string>

#include <thread>
#include <chrono>
#include <deque>
#include <mutex>
#include <random>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "tf/transform_datatypes.h" //To use getYaw function from the quaternion of orientation

#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Odometry.h"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include "laserprocessing.h"

class PfmsSample{

public:
  /*! @brief PfmsSample constructor.
   *
   *  Will take the node handle and initialise the callbacks and internal variables
   */
    PfmsSample(ros::NodeHandle nh);

  /*! @brief PfmsSample destructor.
   *
   *  Will tear down the object
   */
    ~PfmsSample();

    /*! @brief seperate thread.
    *
    *  The main processing thread that will run continously and utilise the data
    *  When data needs to be combined then running a thread seperate to callback will gurantee data is processed
   */
   void seperateThread();

private:
  /*! @brief Odometry Callback
   *
   *  @param nav_msgs::OdometryConstPtr - The odometry message
   *  @note This function and the declaration are ROS specific
   */
    void odomCallback(const nav_msgs::OdometryConstPtr& msg);

 /*! @brief LaserScan Callback
   *
   *  @param sensor_msgs::LaserScanConstPtr - The laserscan message
   *  @note This function and the declaration are ROS specific
   */
    void laserCallback(const sensor_msgs::LaserScanConstPtr& msg);

    /*! @brief We create a marker array from an x,y locatuin supplied
     *
     *  A MarkerArray of a single orange cylinder is created at specified position with a transparency of 50%
     *  The reference frame is world, and it stays on for 2 seconds and within a namespace test.
     *
     *  @param x - location in [m]
     *  @param y - location in [m]
     *  @return markerarray as per specifications
     */
    visualization_msgs::Marker createMarker(double x, double y,double z);


private:
    ros::NodeHandle nh_;
    ros::Subscriber sub1_;
    ros::Subscriber sub2_;

    ros::Publisher viz_pub_; /*!< Publisher for visualisation markers */

    geometry_msgs::Pose robotPose_;
    std::mutex robotPoseMtx_; /*!< Mutex to lock robotPose_ */

    sensor_msgs::LaserScan laserData_;
    std::mutex laserDataMtx_; /*!< Mutex to lock Laser Data */

    unsigned int marker_counter_;

};

