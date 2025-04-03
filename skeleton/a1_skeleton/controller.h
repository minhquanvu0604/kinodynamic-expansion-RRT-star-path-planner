#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "controllerinterface.h"
#include "pipes.h"

class Controller: public ControllerInterface
{
public:

  //Default constructors should set all sensor attributes to a default value
  Controller();

  //See controllerinterface.h for more information
  
  // 1 VIRTUAL
  virtual bool reachGoal(void) = 0;

  // 2 VIRTUAL
  virtual bool setGoal(pfms::geometry_msgs::Point goal) = 0;

  // 3 VIRTUAL
  virtual bool checkOriginToDestination(pfms::nav_msgs::Odometry origin,
                                        pfms::geometry_msgs::Point goal,
                                        double& distance,
                                        double& time,
                                        pfms::nav_msgs::Odometry& estimatedGoalPose) = 0;
                                                                             
  // 4
  pfms::PlatformType getPlatformType(void) override;
  
  // 5 
  virtual double distanceToGoal(void) = 0;

  // 6 
  double timeToGoal(void) override;

  // 7
  bool setTolerance(double tolerance) override;

  // 8
  // double distanceTravelled(void) override;

  // 9
  // double timeInMotion(void) override;

  // 10 
  pfms::nav_msgs::Odometry getOdometry(void) override;

protected:
  Pipes* pipesPtr;

  pfms::PlatformType platformType_;
  
  double distance_;
  double time_;

  pfms::PlatformType type_;
  pfms::nav_msgs::Odometry odo_; // HOW THIS ODO IS RELATED TO PIPE???
  pfms::nav_msgs::Odometry origin_;

  pfms::nav_msgs::Odometry estimatedGoalPose_;

  double tolerance_ = 0;
};

#endif // CONTROLLER_H
