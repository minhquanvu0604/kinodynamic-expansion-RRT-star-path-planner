#ifndef ACKERMAN_H
#define ACKERMAN_H

#include "controller.h"

class Ackerman: public Controller
{
public:

    /**
     * @brief Default constructor
     * 
     * Default constructor should set all sensor attributes to a default value
     * It also initialize a new thread to execute the running of Ackerman
    */
    Ackerman();
    

    /**
     * @brief Setter for goals
     * 
     * @param goals The vector of points as goals
     * @return Whether all goals are reachable, in order supplied
    */
    bool setGoals(std::vector<pfms::geometry_msgs::Point> goals) override;

    /**
     * @brief Checks whether the platform can travel between origin and destination
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

    /**
     * @brief Execute the Ackerman to reach goals
     * 
     * This function is ran in a separate thread to facilitate concurrency of execution
    */
    void executeAckerman();
  
    // Audi's steering, not protected
    double steering_;

    // Some modification for checkOriginToDestination
    // Whether to update the steering_ 
    bool updateSteering_ = false;
    

    /// @brief Whether the distance and time is set to -1 when the goal is unreachable
    bool updateTimeDist_ = false;

    // Fixed throttle value specified by assignment 1 rules
    const double fixThrottle = 0.1;


    // CONSTANT
    // Specification
    static constexpr double STEERING_RATIO = 17.3;
    static constexpr double LOCK_TO_LOCK_REVS = 3.2;
    static constexpr double MAX_STEER_ANGLE = (M_PI*LOCK_TO_LOCK_REVS/STEERING_RATIO); // 0.581104 ~ 33.294806658 degree

    // Steer angle negative -> turn right
    // Steer angle positive -> turn left

    static constexpr double TRACK_WIDTH = 1.638;
    static constexpr double WHEEL_RADIUS = 0.36;
    static constexpr double WHEELBASE = 2.65; // The distance between the robot's right wheels' center point and the robot's left wheels' center point
    static constexpr double MAX_BRAKE_TORQUE = 8000;
    static constexpr double DEFAULT_THROTTLE = 0.1; // Vehicle top speed 2.91 m/s

    static constexpr double CONSTANT_SPEED = 2.91;

};

#endif // ACKERMAN_H
