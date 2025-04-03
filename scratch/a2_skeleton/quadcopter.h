#ifndef QUADCOPTER_H
#define QUADCOPTER_H

#include "controller.h"

class Quadcopter: public Controller
{
public:
  //Default constructor - should set all sensor attributes to a default value
  Quadcopter();

  // /**
  // Run controller in reaching goals - non blocking call
  // */
  // void run(void) override;

  // NO NO NO
  // /**
  // Retrurns platform status (indicating if it is executing a series of goals or idle - waiting for goals)
  // @return platform status
  // */
  // pfms::PlatformStatus status(void) override;
  
  /**
  Setter for goals
  @param goals
  @return all goal reachable, in order supplied
  */
  bool setGoals(std::vector<pfms::geometry_msgs::Point> goals) override;

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
  Getter for distance to be travelled to reach current goal
  @return distance to be travlled to reach current goal [m]
  */
  virtual double distanceToGoal(void) override;

  /**
  Getter for time to reach current goal
  @return time to travel to current goal [s]
  */
  virtual double timeToGoal(void) override;
                                      
  // /**
  // returns distance travelled by platform
  // @return total distance travelled since execution @sa run called with goals supplied
  // */
  // virtual double distanceTravelled(void) override;

  // /**
  // returns total time in motion by platform
  // @return total time in motion since execution @sa run called with goals supplied
  // */
  // virtual double timeTravelled(void) override;

};

#endif // QUADCOPTER_H
