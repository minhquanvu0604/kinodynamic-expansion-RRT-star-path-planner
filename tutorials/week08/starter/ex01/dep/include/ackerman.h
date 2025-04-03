#ifndef ACKERMAN_H
#define ACKERMAN_H

#include "controller.h"

class Ackerman: public Controller
{
public:
  //Default constructor should set all sensor attributes to a default value
  Ackerman();
  ~Ackerman();

  bool reachGoal(void);

  bool setGoal(pfms::geometry_msgs::Point goal);

  bool checkOriginToDestination(pfms::nav_msgs::Odometry origin, pfms::geometry_msgs::Point goal,
                                 double& distance, double& time, pfms::nav_msgs::Odometry& estimatedGoalPose);

  pfms::nav_msgs::Odometry getOdometry(void);

private:
  bool computeSteering(double& steering,double& dist);

private:
  unsigned int ugvSeq_;
  const double steering_ratio_;
  const double lock_to_lock_revs_;
  const double max_steer_angle_;
  const double wheelbase_;
  const double max_break_torque_;
  const double steadyStateV_;
  const double deltaD_;
  double prevD_;

};

#endif // ACKERMAN_H
