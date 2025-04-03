#ifndef LOGGER_H
#define LOGGER_H

#include "ros/ros.h"
#include "geometry_msgs/PoseArray.h"

#include <string>
#include <sstream>
#include <fstream>
#include <vector>

namespace logger {


  /*! @brief Obtain @Points from log file
   *
   *  @param fileName - file containing the points logged
   *  @param points - Vector of geometry_msgs::Point
   *  @return bool - Indicates if file was opened/read corectly
   */
  bool loadPoints(std::string fileName, std::vector<geometry_msgs::Point>& points);

  /*! @brief Save @Points from log file, overwrites the log file
   *
   *  @param fileName - file to save the points
   *  @param points - Vector of geometry_msgs::Point
   *  @return bool - Indicates if file was opened/written to corectly
   */

  bool savePoints(std::string fileName, std::vector<geometry_msgs::Point> points);

  /*! @brief Return string information of vector of points
   *
   *  @param points - vector of geometry_msgs::Point
   *  @return string containing information "x y "
   */
   std::string toString(std::vector<geometry_msgs::Point> points);



};

#endif // LOGGER_H
