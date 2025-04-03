#ifndef LASERPROCESSING_H
#define LASERPROCESSING_H

#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Pose.h>
#include <math.h>

#include "ros/ros.h"
#include "tf/transform_datatypes.h"
#include "heuristicRRTstar/heuristicRRTstar.h"


class LaserProcessing
{
public:

  /*! @brief Default constructor that create LaserProcessing object
   */
  LaserProcessing();


  /*! @brief Constructor that allocates internals
   *
   *  @param[in]    laserScan - laserScan to be processed
   */
  LaserProcessing(sensor_msgs::LaserScan laserScan);


  /*! @brief Setter for laser scan data
   *
   *  @param[in]   laserScan - laserScan to be processed
   */
  void setLaserScan(sensor_msgs::LaserScan laserScan);


  /*! @brief Return indexes in the "ranges" array of sensor_msgs/LaserScan
   *
   *  @param[in|out]   running  control the running status of the ackerman
   * 
   * For the case of obstacle blocking the track, the function set the boolean to false
   */
  std::vector<int> conePointIndex(std::atomic<bool>* running);


  /*! @brief Convert ranges indices to global Cartesian coordinate
   *
   *  @param[in]   conePointIndices  a vector indices returned by @sa conePointIndes
   *  @param[in]   pose  the pose of the ackerman
   */
  std::vector<Point> getPoints(std::vector<int> conePointIndices, geometry_msgs::Pose pose);

  /*! @brief Calculate 
   */
  double cross(const Point &O, const Point &A, const Point &B);

  /*! @brief Create the convex hull of the points provided
   *  
   * @param[in] points - the vector of points to create the convex hull
   * @param[out] - the convex hull
   */
  std::vector<Point> convexHull(std::vector<Point> points);

  /*! @brief Check if the goal is inside the convex hull
   *  
   * @param[in] points - the vector of points to create the convex hull
   * @param[in] p - the goal point
   * @param[out] - whether the goal point is in
   */
  bool pointInConvexPolygon(const std::vector<Point>& hull, Point p);


  /*! @brief Get a number of closest cones with respect to the ackerman
   *  
   * @param[in] points - list of all points
   * @param[in] p - the point representing the ackerman
   * @param[out] - vector of closest points
   */
  std::vector<Point> getClosestPoints(const std::vector<Point>& points, const Point& target);

private:
  // @brief Calculate euclidean distance
  double euclideanDistance(Point pt1, Point pt2);

  // @brief Calculate euclidean distance
  double euclideanDistance(double x1, double y1, double x2, double y2);

  // Return point from laser scan index
  geometry_msgs::Point polarToCart(unsigned int index);

  // Convert from local to global frame
  Point localToGlobal(Point localPoint, geometry_msgs::Pose pose);

  // Normalise the angle to range between -pi and pi
  double normaliseAngle(double theta);

  sensor_msgs::LaserScan laserScan_; //< laserscan data
  geometry_msgs::Pose pose_; //< pose of the car


};

#endif // DETECTCABINET_H