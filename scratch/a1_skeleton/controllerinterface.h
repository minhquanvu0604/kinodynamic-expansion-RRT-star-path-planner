#ifndef CONTROLLERINTERFACE_H
#define CONTROLLERINTERFACE_H

#include <vector>
#include "pfms_types.h"

/*!
 *  \brief     Controller Interface Class
 *  \details
 *  This interface class is used to set all the methods that need to be embodies within any subsequent derived autonomous vehicle controller classes.
 *  The methods noted in interface class are the only methods that will be visible and used for testing the implementation of your code.
 *  \author    Alen Alempijevic
 *  \version   1.01-1
 *  \date      2022-03-08
 *  \pre       none
 *  \bug       none reported as of 2022-03-08
 *  \warning   students MUST NOT change this class (the header file)
 */

class ControllerInterface
{
public:
  ControllerInterface(){};

  /**
  Reach reach goal - execute control to reach goal, blocking call until goal reached or abandoned
  @return goal reached (true - goal reached, false - goal abandoned : not reached)
  */
  virtual bool reachGoal(void) = 0;

  /**
  Setter for goal, the function will update internal variables asscoiated with @sa timeToGoal
  and @sa distanceToGoal
  @return goal reachable
  */
  virtual bool setGoal(pfms::geometry_msgs::Point goal) = 0;

  /**
  Checks whether the platform can travel between origin and destination
  @param[in] origin The origin pose, specified as odometry for the platform
  @param[in] destination The destination point for the platform
  @param[in|out] distance The distance [m] the platform will need to travel between origin and destination. If destination unreachable distance = -1
  @param[in|out] time The time [s] the platform will need to travel between origin and destination, If destination unreachable time = -1
  @param[in|out] estimatedGoalPose The estimated goal pose when reaching goal
  @return bool indicating the platform can reach the destination from origin supplied
  */
  virtual bool checkOriginToDestination(pfms::nav_msgs::Odometry origin,
                                        pfms::geometry_msgs::Point goal,
                                        double& distance,
                                        double& time,
                                        pfms::nav_msgs::Odometry& estimatedGoalPose) = 0;

  /**
  Getter for pltform type
  @return PlatformType
  */
  virtual pfms::PlatformType getPlatformType(void) = 0;

  /**
  Getter for distance to be travelled to reach goal, updates at the platform moves to current goal
  @return distance to be travlled to goal [m]
  */
  virtual double distanceToGoal(void) = 0;

  /**
  Getter for time to reach goal, updates at the platform moves to current goal
  @return time to travel to goal [s]
  */
  virtual double timeToGoal(void) = 0;

  /**
  Set tolerance when reaching goal
  @return tolerance accepted [m]
  */
  virtual bool setTolerance(double tolerance) = 0;

  /**
  returns total distance travelled by platform
  @return total distance travelled since started
  */
  virtual double distanceTravelled(void) = 0;

  /**
  returns total time in motion by platform
  @return total time in motion since started
  */
  virtual double timeInMotion(void) = 0;

  /**
  returns current odometry information
  @return odometry - current pose (x,y,yaw) and velocity (vx,vy)
  */
  virtual pfms::nav_msgs::Odometry getOdometry(void) = 0;

};

#endif // CONTROLLERINTERFACE_H
