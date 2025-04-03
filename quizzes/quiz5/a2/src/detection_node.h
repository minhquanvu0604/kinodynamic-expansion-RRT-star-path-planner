#include <sstream>
#include <iostream>
#include <string>

#include <thread>
#include <chrono>
#include <mutex>
#include <atomic>

#include "ros/ros.h"
#include "tf/transform_datatypes.h" //To use getYaw function from the quaternion of orientation

#include "nav_msgs/Odometry.h"

#include "rangeprocessing.h"

/**
 * This node shows some connections and publishing images
 */


class Detection{

public:
  /*! @brief Detection constructor.
   *
   *  Will take the node handle and initialise the callbacks and internal variables
   */
    Detection(ros::NodeHandle nh);

  /*! @brief Detection destructor.
   *
   *  Will tear down the object
   */
    ~Detection();


  /*! @brief Odometry Callback
   *
   *  @param nav_msgs::OdometryConstPtr - The odometry message
   *  @note This function and the declaration are ROS specific
   */
    void odomCallback(const nav_msgs::OdometryConstPtr& msg);

  /*! @brief range Callback
    *
    *  @param sensor_msgs::RangeConstPtr - The range message
    *  @note This function and the declaration are ROS specific
    */
     void rangeCallback(const sensor_msgs::RangeConstPtr & msg);

  /*! @brief seperate thread.
   *
   *  The main processing thread that will run continously and utilise the data
   *  When data needs to be combined then running a thread seperate to callback will gurantee data is processed
   */

    void seperateThread(); 

private:
    ros::NodeHandle nh_;

    ros::Subscriber sub1_;
    ros::Subscriber sub2_;
    ros::ServiceServer service_;


    struct PoseDataBuffer
    {
        geometry_msgs::Pose pose;
        std::mutex mtx;
    };
    PoseDataBuffer poseDataBuffer_;//! Container for pose data

    struct RangeBuffer
    {
        sensor_msgs::Range range;
        std::mutex mtx;
    };

    RangeBuffer rangeBuffer_;//! Container for image data

};

