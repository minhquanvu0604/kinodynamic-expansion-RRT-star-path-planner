#ifndef CONTROLLERINTERFACE_H
#define CONTROLLERINTERFACE_H

#include <vector>

#include "ros/ros.h"
#include "tf/transform_datatypes.h" //To use getYaw function from the quaternion of orientation
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Point.h"


namespace pfms{
  typedef enum {
    ACKERMAN, /*!< Ackerman based steering ground vehicle */
    QUADCOPTER /*!< Quadcopter */
  } PlatformType; /*!< Platform Types */
}


/*!
 *  \brief     Controller Interface Class
 *  \details
 *  This interface class is used to set all the methods that need to be embodies within any subsequent derived autonomous vehicle controller classes.
 *  The methods noted in interface class are the only methods that will be visible and used for testing the implementation of your code.
 *  \author    Minh Quan Vu
 *  \date      2022-05-18
 *  \pre       none
 *  \bug       none reported as of 2022-04-15
 */

class ControllerInterface
{
public:

  ControllerInterface(){};

  /**
   * Run controller in reaching goals - non blocking call
  */
  virtual void run(void) = 0;
  
  //   /**
  //   Retrurns platform status (indicating if it is executing a series of goals or idle - waiting for goals)
  //   @return platform status
  //   */
  //   virtual pfms::PlatformStatus status(void) = 0;


  // /**
  //  * Setter for goals
  //  * @param goals
  //  * @return all goal reachable, in order supplied
  //  */
  // virtual bool setGoal(geometry_msgs::Point goal) = 0;


  /**
   * Checks whether the platform can travel between origin and destination
   * @param[in] origin The origin pose, specified as odometry for the platform
   * @param[in] destination The destination point for the platform
   * @param[in|out] distance The distance [m] the platform will need to travel between origin and destination. If destination unreachable distance = -1
   * @param[in|out] time The time [s] the platform will need to travel between origin and destination, If destination unreachable time = -1
   * @param[in|out] estimatedGoalPose The estimated goal pose when reaching goal
   * @return bool indicating the platform can reach the destination from origin supplied
  */
  virtual bool checkOriginToDestination(geometry_msgs::Pose origin,
                                    geometry_msgs::Point goal,
                                    double& distance,
                                    double& time,
                                    geometry_msgs::Pose& estimatedGoalPose) = 0;
  /**
  Getter for pltform type
  @return PlatformType
  */
  virtual pfms::PlatformType getPlatformType(void) = 0;

  // /**
  // Getter for distance to be travelled to reach current goal
  // @return distance to be travlled to reach current goal [m]
  // */
  // virtual double distanceToGoal(void) = 0;

  // /**
  // Getter for time to reach current goal
  // @return time to travel to current goal [s]
  // */
  // virtual double timeToGoal(void) = 0;

  // /**
  // Set tolerance when reaching goal
  // @return tolerance accepted [m]
  // */
  // virtual bool setTolerance(double tolerance) = 0;

  // /**
  // returns distance travelled by platform
  // @return total distance travelled since execution @sa run called with goals supplied
  // */
  // virtual double distanceTravelled(void) = 0;

  // /**
  // returns total time in motion by platform
  // @return total time in motion since execution @sa run called with goals supplied
  // */
  // virtual double timeTravelled(void) = 0;

  /**
   * returns current odometry information
   * @return odometry - current odometry
  */
  virtual nav_msgs::Odometry getOdometry(void) = 0;

};






#endif