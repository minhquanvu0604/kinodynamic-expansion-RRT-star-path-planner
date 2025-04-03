#ifndef CONTROLLER_H
#define CONTROLLER_H

#define DEGUG 1

#include "controllerinterface.h"
#include "pipes.h"

class Controller: public ControllerInterface
{
public:
    //Default constructors should set all sensor attributes to a default value
  Controller();
  ~Controller();

  //See controllerinterface.h for more information

    pfms::PlatformType getPlatformType(void);

    bool setTolerance(double tolerance);

    double distanceTravelled(void);

    double timeInMotion(void);

    //////////////////////////////////////////////////
    virtual bool setGoal(pfms::geometry_msgs::Point goal) = 0;

    virtual bool reachGoal(void) = 0;

    double distanceToGoal(void);

    double timeToGoal(void);

    virtual bool checkOriginToDestination(pfms::nav_msgs::Odometry origin, pfms::geometry_msgs::Point goal,
                                   double& distance, double& time,  pfms::nav_msgs::Odometry& estimatedGoalPose) = 0;

    virtual pfms::nav_msgs::Odometry getOdometry(void) = 0;

protected:
    void printGoal(void);

private:



protected:
    pfms::PlatformType type_;
    Pipes* pipes_;
    pfms::nav_msgs::Odometry odo_;
    pfms::geometry_msgs::Point goal_;
    double tolerance_;
    double distanceTravelled_;
    double timeInMotion_;
    double distToGoal_;
    double timeToGoal_;

private:




};

#endif // CONTROLLER_H
