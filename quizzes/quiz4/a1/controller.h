#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "controllerinterface.h"
#include <cmath>
#include <pipes.h>

//! Information about the goal for the platform
struct GoalStats {
    //! location of goal
    pfms::geometry_msgs::Point location;
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
  bool setGoal(pfms::geometry_msgs::Point goal);
  bool setTolerance(double tolerance);
  double distanceToGoal(void);
  double timeToGoal(void);
  double distanceTravelled(void);
  double timeInMotion(void);
  pfms::PlatformType getPlatformType(void);

  /**
   * Updates the internal odometry
   *
   * Sometimes the pipes can give all zeros on opening, this has a little extra logic to ensure only valid data is
   * accepted
   */
  pfms::nav_msgs::Odometry getOdometry(void);


protected:
  /**
   * Checks if the goal has been reached.
   *
   * Update own odometry before calling!
   * @return true if the goal is reached
   */
  bool goalReached();



  Pipes* pipesPtr_; //!< The pipe to communicate
  pfms::nav_msgs::Odometry odo_;//!< The current pose of platform

  //stats
  GoalStats goal_;
  pfms::PlatformType type_;


  double distance_travelled_; //!< Total distance travelled for this program run
  double time_travelled_; //!< Total time spent travelling for this program run
  double tolerance_; //!< Radius of tolerance
  long unsigned int cmd_pipe_seq_; //!<The sequence number of the command

};

#endif // CONTROLLER_H
