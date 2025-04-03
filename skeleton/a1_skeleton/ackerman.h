#ifndef ACKERMAN_H
#define ACKERMAN_H

#include "controller.h"

class Ackerman: public Controller
{
public:
  //Default constructor should set all sensor attributes to a default value
  Ackerman();

  // 1
  bool reachGoal(void) override;

  // 2
  bool setGoal(pfms::geometry_msgs::Point goal) override;

  // 3
  bool checkOriginToDestination(pfms::nav_msgs::Odometry origin,
                                        pfms::geometry_msgs::Point goal,
                                        double& distance,
                                        double& time,
                                        pfms::nav_msgs::Odometry& estimatedGoalPose) override;
 
  // 5 
  double distanceToGoal(void) override;

protected:
  double turningRadius_; 

private:
  // VARIABLES
  pfms::geometry_msgs::Point goal_;


  // CONSTANT
  // Specification
  static const double STEERING_RATIO = 17.3;
  static const double LOCK_TO_LOCK_REVS = 3.2;
  static const double MAX_STEER_ANGLE = (M_PI*LOCK_TO_LOCK_REVS/STEERING_RATIO); // 0.581104 ~ 33.294806658 degree
  // Steer angle negative -> turn right
  // Steer angle positive -> turn left
  static const double TRACK_WIDTH = 1.638;
  static const double WHEEL_RADIUS = 0.36;
  static const double WHEELBASE = 2.65; // The distance between the robot's right wheels' center point and the robot's left wheels' center point
  static const double MAX_BRAKE_TORQUE = 8000;
  static const double DEFAULT_THROTTLE = 0.1; // Vehicle top speed 2.91 m/s

  static const double CONSTANT_SPEED = 2.91;

  // Calculate the minimum turning radius, which is based on the maximum steering angle
  double r_min =  WHEELBASE / tan(MAX_STEER_ANGLE); // 4.044


  // FUNCTIONS
  std::pair<double, double> turningCentre(double ini_x, double ini_y, double goal_x, double goal_y, double yaw);
  std::tuple<double, double, double> headingLine (double yaw, double x, double y);

  double euclideanDistance(double x1, double y1, double x2, double y2);
  std::tuple<double, double, double> headingLine (double x, double y, double yaw);
  std::tuple<double, double, double> lineFromPoints(double x1, double y1, double x2, double y2);
  std::tuple<double, double, double> perpendicularBisector(double x1, double y1, double x2, double y2);
  std::tuple<double, double, double> lineFromPointAndPerpendicularLine(double x, double y, std::tuple<double, double, double> perp_line);
  std::pair<double, double> intersectionPoint(std::tuple<double, double, double> line1, std::tuple<double, double, double> line2);

  double normalizeAngle(double angle);
  double turnDirection(double current_x, double current_y, double current_yaw, double goal_x, double goal_y, double turningAngle);

};

#endif // ACKERMAN_H
