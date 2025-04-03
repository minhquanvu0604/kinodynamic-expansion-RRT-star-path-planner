#ifndef ACKERMAN_H
#define ACKERMAN_H

#include <cmath>
#include "controller.h"

class Ackerman: public Controller
{
public:

  // Default constructor should set all sensor attributes to a default value
  Ackerman();

  /**
  Reach reach goal - execute control to reach goal, blocking call until goal reached or abandoned
  @return goal reached (true - goal reached, false - goal abandoned : not reached)
  */
  bool reachGoal(void) override;

  /**
  Setter for goal, the function will update internal variables asscoiated with @sa timeToGoal
  and @sa distanceToGoal
  @return goal reachable
  */
  bool setGoal(pfms::geometry_msgs::Point goal) override;

  /**
  Checks whether the platform can travel between origin and destination
  @param[in] origin The origin pose, specified as odometry for the platform
  @param[in] destination The destination point for the platform
  @param[in|out] distance The distance [m] the platform will need to travel between origin and destination. If destination unreachable distance = -1
  @param[in|out] time The time [s] the platform will need to travel between origin and destination, If destination unreachable time = -1
  @param[in|out] estimatedGoalPose The estimated goal pose when reaching goal
  @return bool indicating the platform can reach the destination from origin supplied
  */
  bool checkOriginToDestination(pfms::nav_msgs::Odometry origin,
                                        pfms::geometry_msgs::Point goal,
                                        double& distance,
                                        double& time,
                                        pfms::nav_msgs::Odometry& estimatedGoalPose) override;

  /**
  returns total distance travelled by platform
  @return total distance travelled since started
  */
  double distanceTravelled(void) override;

  /**
  returns current odometry information
  @return odometry - current pose (x,y,yaw) and velocity (vx,vy)
  */
  pfms::nav_msgs::Odometry getOdometry(void) override;


private:
  /**
  @brief travelled
  The distance the platform has travelled from program start.
  Can be updated at every time step by @sa getOdometry
  */
  double distanceTravelled_;

  /**
  @brief Steering
  Simulates the steering in revolution of the car's steering wheel.
  Used to feed commands into the Pipe
  */
  double steering_;

  // Goal in local frame of reference, updated by checkOriginToDestination
  pfms::geometry_msgs::Point localGoal_{0, 0}; 

  // Goal in global frame of reference, updated by setGoal
  pfms::geometry_msgs::Point globalGoal_{0 ,0}; 

  // Fixed throttle value specified by assignment 1 rules
  const double fixThrottle = 0.1;


  // CONSTANT

  // Specification

  static constexpr double STEERING_RATIO = 17.3;
  static constexpr double LOCK_TO_LOCK_REVS = 3.2;
  static constexpr double MAX_STEER_ANGLE = (M_PI*LOCK_TO_LOCK_REVS/STEERING_RATIO); // 0.581104 ~ 33.294806658 degree

  // Steer angle negative -> turn right
  // Steer angle positive -> turn left

  static constexpr double TRACK_WIDTH = 1.638;
  static constexpr double WHEEL_RADIUS = 0.36;
  static constexpr double WHEELBASE = 2.65; // The distance between the robot's right wheels' center point and the robot's left wheels' center point
  static constexpr double MAX_BRAKE_TORQUE = 8000;
  static constexpr double DEFAULT_THROTTLE = 0.1; // Vehicle top speed 2.91 m/s

  static constexpr double CONSTANT_SPEED = 2.91;


  // HELPER FUNCTIONS

  /**
  @brief Convert to local frame of reference
  Convert the goal point from global frame of reference to local frame of reference
  */
  pfms::geometry_msgs::Point globalToLocalFrame(pfms::geometry_msgs::Point goal, pfms::geometry_msgs::Point robotPosition, double robotYaw);
};

#endif // ACKERMAN_H
