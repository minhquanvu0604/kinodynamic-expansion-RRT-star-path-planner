#ifndef QUADCOPTER_H
#define QUADCOPTER_H

#include "controller.h"

class Quadcopter: public Controller
{
public:

    /**
     * @brief Default constructor
     * 
     * Default constructor should set all sensor attributes to a default value
     * It also initialize a new thread to execute the running of Quadcopter
    */
    Quadcopter();

    
    /**
     * @brief Setter for goals
     * 
     * @param goals The vector of points as goals
     * @return Whether all goals are reachable, in order supplied
    */
    bool setGoals(std::vector<pfms::geometry_msgs::Point> goals) override;


    /**
     * Checks whether the platform can travel between origin and destination
     * 
     * 
     * 
     * @param[in] origin The origin pose, specified as odometry for the platform
     * @param[in] destination The destination point for the platform
     * @param[in|out] distance The distance [m] the platform will need to travel between origin and destination. If destination unreachable distance = -1
     * @param[in|out] time The time [s] the platform will need to travel between origin and destination, If destination unreachable time = -1
     * @param[in|out] estimatedGoalPose The estimated goal pose when reaching goal
     * @return bool indicating the platform can reach the destination from origin supplied
    */
    bool checkOriginToDestination(pfms::nav_msgs::Odometry origin,
                                          pfms::geometry_msgs::Point goal,
                                          double& distance,
                                          double& time,
                                          pfms::nav_msgs::Odometry& estimatedGoalPose) override;

private:
    void executeQuadcopter();

    // Constants
    const double recommendedVelocity = 0.4;

};

#endif // QUADCOPTER_H
