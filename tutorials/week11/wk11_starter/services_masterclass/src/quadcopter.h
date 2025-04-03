#ifndef QUADCOPTER_H
#define QUADCOPTER_H

#include "controller.h"

//We include messages types for quadcopter
#include "geometry_msgs/Twist.h"
#include "std_msgs/Empty.h"
#include "std_srvs/SetBool.h"

namespace quadcopter{
    typedef enum {
      GROUNDED, /*!< On ground, not taken off */
      IDLE, /*!< Stationary with no goal */
      RUNNING, /*!< Executing a motion */
      TAKEOFF, /*!< taking off */
      LANDING /*!< landing */
    } PlatformStatus; /*!< Platform Status */
}

//! UAV drone platform controller
class Quadcopter: public Controller
{
public:
  Quadcopter(ros::NodeHandle nh);

  ~Quadcopter();

  void reachGoal(void);
  /**
   * Calculates the angle needed for the quadcopter to reach a goal.
   * @return Always true - quadcopter has no unreachable goals.
   */
  bool calcNewGoal(void);

  bool checkOriginToDestination(nav_msgs::Odometry origin, geometry_msgs::Point goal,
                                 double& distance, double& time,
                                 nav_msgs::Odometry& estimatedGoalPose);


  bool request(std_srvs::SetBool::Request  &req,
             std_srvs::SetBool::Response &res);

private:

  void sendCmd(double turn_l_r, double move_l_r, double move_u_d, double move_f_b);

  //! Angle required for quadcopter to have a straight shot at the goal
  double target_angle_ = 0;
  bool liftoff_;

  const double TARGET_SPEED;
  const double TARGET_HEIGHT_TOLERANCE;

  ros::Publisher pubCmdVel_;
  ros::Publisher pubTakeOff_;

  ros::ServiceServer service_;  

};

#endif // QUADCOPTER_H
