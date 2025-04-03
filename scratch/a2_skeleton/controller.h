#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "controllerinterface.h"
#include "pipes.h"

//See controllerinterface.h for more information
class Controller: public ControllerInterface
{
public:

  //Default constructors should set all sensor attributes to a default value
  Controller();


  // /**
  // Run controller in reaching goals - non blocking call
  // */
  // virtual void run(void) = 0;


  /**
  Retrurns platform status (indicating if it is executing a series of goals or idle - waiting for goals)
  @return platform status
  */
  virtual pfms::PlatformStatus status(void) override;


  /**
  Setter for goals
  @param goals
  @return all goal reachable, in order supplied
  */
  virtual bool setGoals(std::vector<pfms::geometry_msgs::Point> goals) = 0;


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
  pfms::PlatformType getPlatformType(void) override;


  /**
  Getter for distance to be travelled to reach current goal
  @return distance to be travlled to reach current goal [m]
  */
  virtual double distanceToGoal(void) = 0;


  /**
  Getter for time to reach current goal
  @return time to travel to current goal [s]
  */
  virtual double timeToGoal(void) = 0;


  /**
  Set tolerance when reaching goal
  @return tolerance accepted [m]
  */
  bool setTolerance(double tolerance) override;


//   /**
//   returns distance travelled by platform
//   @return total distance travelled since execution @sa run called with goals supplied
//   */
//   virtual double distanceTravelled(void) = 0;


//   /**
//   returns total time in motion by platform
//   @return total time in motion since execution @sa run called with goals supplied
//   */
//   virtual double timeTravelled(void) = 0;


  /**
  returns current odometry information
  @return odometry - current odometry
  */
  virtual pfms::nav_msgs::Odometry getOdometry(void) override;


protected:

  /**
  @brief Pointer to a Pipes object
  This variable facilitates the communication between ROS and student program.
  Interfacing through this pointer abstracts the underlying work of ROS.
  */
  Pipes* pipesPtr_;

  /**
  @brief Controller odometry 
  This variable contains the instantaneous odometry of the platform which can be read and sent through the Pipe.
  */
  pfms::nav_msgs::Odometry odo_; 
 
  /**
  @brief Type of platform 
  Enum type of the type of platform the controller is. 
  Can take either value ACKERMAN or QUADCOPTER.
  */
  pfms::PlatformType platformType_;

  /**
  @brief Tolerance
  The tolerance value of the straight distance from the platform to the goal.
  Used to judge whether the platforms have accomplished reaching the goals.
  */
  double tolerance_;

  /**
  @brief Distance to goal
  The distance to goal point, updated in @sa reachGoal where new goal is set.
  */
  double distanceToGoal_;
 
  /**
  @brief Time to goal
  The estimated time to reach the upcoming goal point, updated in @sa reachGoal where new goal is set.
  */
  double timeToGoal_;

  /**
  @brief Instant linear distnace to goal
  The straight distance to goal point, updated in @sa getOdometry and can be called every time step.
  */
  double instantDistanceToGoal_;

  /**
  @brief Distance travelled
  The distance the platform has travelled from program start.
  Can be updated at every time step by @sa getOdometry
  */
  double distanceTravelled_;

  /**
  @brief A private copy of goals
  A vector of Point objects
  */
  std::vector<pfms::geometry_msgs::Point> goals_; 

  /**
  @brief Platform status
  Status of the platform, either IDLE, RUNNING, TAKEOFF or LANDING.
  TAKEOFF and LANDING are for the UAV only.
  */
  pfms::PlatformStatus platformStatus_;


  // --------------// HELPER FUNCTION // --------------------------------------------------------//

  /**
  @brief Convert to local frame 
  Convert the goal point from global frame of reference to local frame of robot
  */
  pfms::geometry_msgs::Point globalToLocalFrame(pfms::geometry_msgs::Point goal, pfms::geometry_msgs::Point robotPosition, double robotYaw);

  /**
  @brief Euclidean distance
  Calculate the distance between 2 points given their coordinates
  @return the distance between 2 points 
  */
  double euclideanDistance(double x1, double y1, double x2, double y2);

};



#endif // CONTROLLER_H
