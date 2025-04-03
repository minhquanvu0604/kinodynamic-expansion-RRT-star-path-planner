#ifndef ACKERMAN_H
#define ACKERMAN_H

#include <atomic>
#include "controller.h"
#include "std_srvs/SetBool.h"


class Ackerman : public Controller 
{
public:

    /**
     * @brief Default constructor
     * 
     * Default constructor should set all sensor attributes to a default value
     * It also initialize a new thread to execute the running of Ackerman
    */
    Ackerman(ros::NodeHandle nh);

    // /**
    //  * Check if goals are with in the road and reachable
    //  * @param goals
    //  * @return all goal reachable, in order supplied
    //  */
    // virtual bool checkGoal(geometry_msgs::Point goal);

    /**
     * Checks whether the platform can travel between origin and destination
     * @param[in] origin The origin pose, specified as odometry for the platform
     * @param[in] destination The destination point for the platform
     * @param[in|out] distance The distance [m] the platform will need to travel between origin and destination. If destination unreachable distance = -1
     * @param[in|out] time The time [s] the platform will need to travel between origin and destination, If destination unreachable time = -1
     * @param[in|out] estimatedGoalPose The estimated goal pose when reaching goal
     * @return bool indicating the platform can reach the destination from origin supplied
    */
    virtual bool checkOriginToDestination(geometry_msgs::Pose origin,
                                    geometry_msgs::Point goal,
                                    double& distance,
                                    double& time,
                                    geometry_msgs::Pose& estimatedGoalPose);
    
private:

    /**
     * Callback function for goal
     */
    void goalCallback (const geometry_msgs::PoseArray::ConstPtr& msg);
    
    bool request(std_srvs::SetBool::Request  &req,
             std_srvs::SetBool::Response &res);
             
    void sendCmd(double brake, double steering, double throttle);

    /**
     * @brief Execute the Ackerman to reach goals
     * 
     * This function is ran in a separate thread to facilitate concurrency of execution
    */
    void executeAckerman();
    void executeADVAckerman();

    void reachGoal();

    void autopilot();

    void braking();

    visualization_msgs::MarkerArray createConesMarker(std::vector<Point> goals);
    visualization_msgs::MarkerArray createGoalMarker(geometry_msgs::PoseArray goals);

    /**
     * @brief Convert to local frame 
     * 
     * Convert the goal point from global frame of reference to local frame of robot
    */
    geometry_msgs::Point globalToLocalFrame(geometry_msgs::Point goal, geometry_msgs::Point robotPosition, double robotYaw);

    // Audi's steering, not protected
    double steering_;

    // Fixed throttle value specified by assignment 1 rules
    const double fixThrottle = 0.1;

    ros::Publisher pubSteering_;
    ros::Publisher pubThrottle_;
    ros::Publisher pubBrake_;

    // CONSTANT
    // Specification
    static constexpr double STEERING_RATIO = 17.3;
    static constexpr double LOCK_TO_LOCK_REVS = 3.2;
    static constexpr double MAX_STEER_ANGLE = (M_PI*LOCK_TO_LOCK_REVS/STEERING_RATIO); // 0.581104 ~ 33.294806658 degree

    // Steer angle negative -> turn right
    // Steer angle positive -> turn left

    static constexpr double TRACK_WIDTH = 1.638;
    static constexpr double WHEEL_RADIUS = 0.36;
    static constexpr double WHEELBASE = 2.65; 
    static constexpr double MAX_BRAKE_TORQUE = 8000;
    static constexpr double DEFAULT_THROTTLE = 0.1; // Vehicle top speed 2.91 m/s

    static constexpr double CONSTANT_SPEED = 2.91;

    // Empirical constants for brake tuning 
    // Progress to the next goal when this speed lowered to this value 
    static constexpr double kMaxSpeedToProgressToNextGoal_ = 0.05;    
    // Empirical value of the minimum brake signal fed to the Pipe
    static constexpr double kMinBrake_ = 5740;
    static constexpr double kBrakeThresholdDefault = 0.3;
    double brakeThreshold_;
};

#endif // ACKERMAN_H

