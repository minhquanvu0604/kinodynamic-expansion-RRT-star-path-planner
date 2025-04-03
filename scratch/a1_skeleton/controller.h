#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "controllerinterface.h"
#include "pipes.h"

/*!
 *  \brief     Controller Class
 *  \details
 *  This class gives implementation to some functions in interface class Controller Interface 
 *  \author    Minh Quan Vu
 *  \version   1.0
 *  \date      2022-03-28
 *  \pre       none
 *  \bug       none reported as of 2022-03-28
 *  \warning   none
 */
class Controller: public ControllerInterface
{
public:

  //Default constructors should set all sensor attributes to a default value
  Controller();

  ~Controller();

  //See controllerinterface.h for more information
  
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
  pfms::PlatformType getPlatformType(void) override;
  
  /**
  Getter for distance to be travelled to reach goal, updates at the platform moves to current goal
  @return distance to be travlled to goal [m]
  */
  double distanceToGoal(void);

  /**
  Getter for time to reach goal, updates at the platform moves to current goal
  @return time to travel to goal [s]
  */
  double timeToGoal(void) override;

  /**
  Set tolerance when reaching goal
  @return tolerance accepted [m]
  */
  bool setTolerance(double tolerance) override;

  /**
  returns total distance travelled by platform
  @return total distance travelled since started
  */
  virtual double distanceTravelled(void) = 0;

  /**
  returns total time in motion by platform
  @return total time in motion since started
  */
  double timeInMotion(void) override;

  /**
  returns current odometry information
  @return odometry - current pose (x,y,yaw) and velocity (vx,vy)
  */
  virtual pfms::nav_msgs::Odometry getOdometry(void) = 0;

protected:

  /**
  @brief Pointer to a Pipes object
  This variable facilitates the communication between ROS and student program.
  Interfacing through this pointer abstracts the underlying work of ROS.
  */
  Pipes* pipesPtr;

  /**
  @brief Controller odometry 
  This variable contains the odometry of the platform which can be read and sent through the Pipe.
  */
  pfms::nav_msgs::Odometry odo_; 
 
  /**
  @brief Type of platform 
  Enum type of the type of platform the controller is. 
  Can take either value ACKERMAN or QUADCOPTER.
  */
  pfms::PlatformType platformType_;

  /**
  @brief Estimated goal pose
  The tolerance value of the straight distance from the platform to the goal.
  Used to judge whether the platforms have accomplished reaching the goals.
  */
  pfms::nav_msgs::Odometry estimatedGoalPose_; 
 
  /**
  @brief Tolerance
  The tolerance value of the straight distance from the platform to the goal.
  Used to judge whether the platforms have accomplished reaching the goals.
  */
  double tolerance_ = 0.5;

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
  double instantLinearDistanceToGoal_;
 
  /**
  @brief Total time that platform has been moving.
  Updated in @sa reachGoal, calculated as the sum of time intervals of executions of @sa reachGoal 
  */
  double timeInMotion_;

  /**
  @brief Euclidean distance
  Calculate the distance between 2 points given their coordinates
  @return the distance between 2 points 
  */
  double euclideanDistance(double x1, double y1, double x2, double y2);
  
  /**
  @brief Normalize angle
  Normalize the input angle to the range of -π (M_PI) to π.
  @return the normalized angle
  */
  double normalizeAngle(double angle);
  
};

#endif // CONTROLLER_H





