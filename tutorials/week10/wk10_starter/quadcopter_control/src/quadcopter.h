#ifndef QUADCOPTER_H
#define QUADCOPTER_H

#include "controller.h"

//! UAV drone platform controller
class Quadcopter: public Controller
{
public:
  Quadcopter(ros::NodeHandle nh);

  ~Quadcopter();

  bool reachGoal(void);
  /**
   * Calculates the angle needed for the quadcopter to reach a goal.
   * @return Always true - quadcopter has no unreachable goals.
   */
  bool calcNewGoal(void);

  bool checkOriginToDestination(nav_msgs::Odometry origin, geometry_msgs::Point goal,
                                 double& distance, double& time,
                                 nav_msgs::Odometry& estimatedGoalPose);

//  pfms::PlatformType getPlatformType(void);

private:

  void sendCmd(double turn_l_r, double move_l_r, double move_u_d, double move_f_b);

  //! Angle required for quadcopter to have a straight shot at the goal
  double target_angle_ = 0;
  bool liftoff_;

  const double TARGET_SPEED;
  const double TARGET_HEIGHT;
  const double TARGET_HEIGHT_TOLERANCE;

};

#endif // QUADCOPTER_H
