#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "controllerinterface.h"
#include <cmath>

//Instead of Pipes now we need to use Ros communication machanism and messages
//#include <pipes.h>
#include "ros/ros.h"
#include "tf/transform_datatypes.h" //To use getYaw function from the quaternion of orientation


//! Information about the goal for the platform
struct GoalStats {
    //! location of goal
    //pfms::geometry_msgs::Point location;
    geometry_msgs::Point location;

    //! distance to goal
    double distance;
    //! time to goal
    double time;
};

/**
 * \brief Shared functionality/base class for platform controllers
 *
 * Platforms need to implement:
 * - Controller::calcNewGoal (and updating GoalStats)
 * - ControllerInterface::reachGoal (and updating PlatformStats)
 * - ControllerInterface::checkOriginToDestination
 * - ControllerInterface::getPlatformType
 * - ControllerInterface::getOdometry (and updating PlatformStats.odo)
 */
class Controller: public ControllerInterface
{
public:
  /**
   * Default Controller constructor, sets odometry and metrics to initial 0
   */
  Controller();

  /**
   * Instructs the underlying platform to recalcuate a goal, and set any internal variables as needed
   *
   * Called when goal or tolerance changes
   * @return Whether goal is reachable by the platform
   */
  virtual bool calcNewGoal() = 0;

  //ControllerInterface functions (all doxygen comments in the files)
  //bool setGoal(pfms::geometry_msgs::Point goal);
  bool setGoal(geometry_msgs::Point goal);

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


  //pfms::PlatformType getPlatformType(void);

  bool setTolerance(double tolerance);

  double distanceTravelled(void);

  double timeInMotion(void);

  double distanceToGoal(void);

  double timeToGoal(void);

  /**
   * Updates the internal odometry
   *
   * Sometimes the pipes can give all zeros on opening, this has a little extra logic to ensure only valid data is
   * accepted
   */
  //pfms::nav_msgs::Odometry getOdometry(void);
  nav_msgs::Odometry getOdometry(void);


protected:
  /**
   * Checks if the goal has been reached.
   *
   * Update own odometry before calling!
   * @return true if the goal is reached
   */
  bool goalReached();

  nav_msgs::Odometry odo_;//!< The current pose of platform

  //stats
  GoalStats goal_;

  double distance_travelled_; //!< Total distance travelled for this program run
  double time_travelled_; //!< Total time spent travelling for this program run
  double tolerance_; //!< Radius of tolerance
  long unsigned int cmd_pipe_seq_; //!<The sequence number of the command

  //Instead of Pipes now we use ROS communication mechanism
  //Pipes* pipesPtr_; //!< The pipe to communicate
  ros::NodeHandle nh_;
  ros::Subscriber sub1_;
  ros::Subscriber sub2_;



};

#endif // CONTROLLER_H
