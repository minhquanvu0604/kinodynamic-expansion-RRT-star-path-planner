#ifndef RANGEPROCESSING_H
#define RANGEPROCESSING_H

#include <sensor_msgs/Range.h>
#include <geometry_msgs/Point.h>
#include <math.h>
#include "tf/transform_datatypes.h"

namespace range_processing {

  /*! TASK 1
   * @brief Return height of object below the quadcopter (the pose of quadcopter is provided)
   *
   * The height is indicated by the lengt/height of the cone (we need to use the position of the drone (which is also the pose of the sensor)
   * and the range to compute the height of whataever is under the drone (heigth from ground)
   * @param[in] - The pose of the sensor
   * @param[in|out] - The height of object (if one is detected)
   * @return returns true if an object is detected
   */
   bool detectHeightOfObject(sensor_msgs::Range rangeScan, geometry_msgs::Pose pose, double &height);


   /*! TASK 2
    * @brief Return position of person below the quadcopter (the pose of quadcopter is provided)
    *
    * The position should be the location of the cone (which can be computed as the average laser readings belonging to the cone)
    * As the cone is circular, the laser will pick up a few readings at the location of the cone.
    * @param[in] - The range sensor scan
    * @param[in] - The pose of the sensor
    * @param[in] - The height of a person
    * @param[in|out] - The person position (if one is detected)
    * @return returns true if an object is detected
    */
    bool detectPerson(sensor_msgs::Range rangeScan, geometry_msgs::Pose pose, double personHeight, geometry_msgs::Point& location);

}

#endif // RANGEPROCESSING_H
