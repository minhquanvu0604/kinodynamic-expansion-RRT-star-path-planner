#include "laserprocessing.h"
#include <algorithm>
#include <numeric>

using namespace std;

LaserProcessing::LaserProcessing(sensor_msgs::LaserScan laserScan):
    laserScan_(laserScan)
{
}

//! @todo
//! TASK 1 - Refer to Header file for full description
geometry_msgs::Point LaserProcessing::closestPoint()
{
  geometry_msgs::Point pt;

  float nearestDistance = laserScan_.range_max;
  unsigned int nearestID = 0;

  for (unsigned int i = 0; i < laserScan_.ranges.size(); i++){
    if (laserScan_.ranges.at(i) < nearestDistance){
      nearestDistance = laserScan_.ranges.at(i);
      nearestID = i;
    }
  }

  float nearestAngle = laserScan_.angle_min + nearestID * laserScan_.angle_increment;

  pt.x = nearestDistance * cos(nearestAngle);
  pt.y = nearestDistance * sin(nearestAngle);
  pt.z = 0;


  return pt;
}


void LaserProcessing::newScan(sensor_msgs::LaserScan laserScan){
    laserScan_=laserScan;
}

