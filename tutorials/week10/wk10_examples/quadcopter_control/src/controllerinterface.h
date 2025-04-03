#ifndef CONTROLLERINTERFACE_H
#define CONTROLLERINTERFACE_H

#include <vector>

//Instead of Pipes now we need to use Ros messages
//#include <pfms_types.h>
#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Point.h"

/*!
 *  \brief     Controller Interface Class for ROS (modified)
 *  \details
 *  This interface class is used to set all the methods that need to be embodies within any subsequent derived autonomous vehicle controller classes.
 *  The methods noted in interface class are the only methods that will be visible and used for testing the implementation of your code.
 *  \author    Alen Alempijevic
 *  \warning   With small modifications to be ROS compliant, removed type (which you would need now to declare in a header)
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

  // /**
  // Setter for goal, the function will update internal variables asscoiated with @sa timeToGoal
  // and @sa distanceToGoal
  // @return goal reachable
  // */
  //We would now have to sacrifice having a return value to have a setGoal
  //At week 10 we do not know about services (which allow us to retrun value
  //So to allow to set a goal via topic we forfit having areturn value for now
  //At week 11 you can replace this with a service  
  // virtual bool setGoal(geometry_msgs::Point goal) = 0;
  virtual void setGoal(const geometry_msgs::Point::ConstPtr& msg) = 0; 
  

  /**
  Checks whether the platform can travel between origin and destination
  @param[in] origin The origin pose, specified as odometry for the platform
  @param[in] destination The destination point for the platform
  @param[in|out] distance The distance [m] the platform will need to travel between origin and destination. If destination unreachable distance = -1
  @param[in|out] time The time [s] the platform will need to travel between origin and destination, If destination unreachable time = -1
  @param[in|out] estimatedGoalPose The estimated goal pose when reaching goal
  @return bool indicating the platform can reach the destination from origin supplied
  */
  virtual bool checkOriginToDestination(nav_msgs::Odometry origin,
                                        geometry_msgs::Point goal,
                                        double& distance,
                                        double& time,
                                        nav_msgs::Odometry& estimatedGoalPose) = 0;

   // /**
  // Getter for pltform type
  // @return PlatformType
  // */
  // virtual pfms::PlatformType getPlatformType(void) = 0;

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
  virtual nav_msgs::Odometry getOdometry(void) = 0;

};

#endif // CONTROLLERINTERFACE_H
