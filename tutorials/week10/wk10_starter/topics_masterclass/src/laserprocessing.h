#ifndef LASERPROCESSING_H
#define LASERPROCESSING_H

#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Pose.h>
#include <math.h>
#include "tf/transform_datatypes.h"

#include <Eigen/Dense>
#include <Eigen/StdVector>

class LaserProcessing
{
public:
  /*! @brief Constructor that allocates internals
   *
   *  @param[in]    laserScan - laserScan to be processed
   */
  LaserProcessing(sensor_msgs::LaserScan laserScan);

  /*! TASK1
   * @brief Determines the closest point to robot, in it's local reference frame
   *
   * @return location of closest point
   */
   geometry_msgs::Point closestPoint();


  /*! @brief Accepts a new laserScan
   *  @param[in]    laserScan  - laserScan to be processed
   */
  void newScan(sensor_msgs::LaserScan laserScan);

private:
  sensor_msgs::LaserScan laserScan_;
};

#endif // LASERPROCESSING_H
