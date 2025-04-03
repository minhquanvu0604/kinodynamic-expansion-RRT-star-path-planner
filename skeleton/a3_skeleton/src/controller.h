#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <thread>
#include <condition_variable>
#include <geometry_msgs/PoseArray.h>
#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/MarkerArray.h>

#include "controllerinterface.h"
#include "laserprocessing.h"


class Controller : public ControllerInterface
{
public:

    Controller();

    ~Controller();

protected:
    /**
     * Run controller in reaching goals - non blocking call
     */
    void run(void);

    //   /**
    //   Retrurns platform status (indicating if it is executing a series of goals or idle - waiting for goals)
    //   @return platform status
    //   */
    //   virtual pfms::PlatformStatus status(void) = 0;

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
                                    geometry_msgs::Pose& estimatedGoalPose) = 0;

    /**
     * Getter for pltform type
     * @return PlatformType
     */
    pfms::PlatformType getPlatformType(void);

    // /**
    // Getter for distance to be travelled to reach current goal
    // @return distance to be travlled to reach current goal [m]
    // */
    // virtual double distanceToGoal(void) = 0;

    // /**
    // Getter for time to reach current goal
    // @return time to travel to current goal [s]
    // */
    // virtual double timeToGoal(void) = 0;

    /**
     * Set tolerance when reaching goal
     * @return tolerance accepted [m]
    */
    virtual bool setTolerance(double tolerance);

    // /**
    // returns distance travelled by platform
    // @return total distance travelled since execution @sa run called with goals supplied
    // */
    // virtual double distanceTravelled(void) = 0;

    // /**
    // returns total time in motion by platform
    // @return total time in motion since execution @sa run called with goals supplied
    // */
    // virtual double timeTravelled(void) = 0;

    /**
     * returns current odometry information
     * @return odometry - current odometry
     */
    virtual nav_msgs::Odometry getOdometry(void);


    /**
     * @brief Tolerance
     * 
     * The tolerance value of the straight distance from the platform to the goal
     * Used to judge whether the platforms have accomplished reaching the goals
    */
    double tolerance_;
    
    ros::NodeHandle nh_;

    ros::Subscriber subOdom_;
    ros::Subscriber subGoal_;
    ros::Subscriber subLaser_;

    ros::Publisher pubMarker_;
    // ros::Publisher pubMark_;

    ros::ServiceServer service_;  

    bool advanced_;

    long id_;

    // static constexpr double kRateLimit_ = 5.0;

    /**
     * Goal ahead provided by laser processing in D/HD mode
    */
    geometry_msgs::Point goal_;

    /**
     * @brief Goals array provided in Pass/Credit 
    */
    geometry_msgs::PoseArray demandGoals_;

    std::mutex mtxDemandGoals_;

    std::mutex mtxLaser_;



    bool demandGoalSet_;
    
    /**
     * @brief Vector of thread objects
     * 
     * Contains the threads which execute different platforms
     * Each element is corresponding to a platform
    */
    std::vector<std::thread> threads_;

    /**
     * @brief Conditional variable  
     * 
     * Used to synchronize the running of platforms after certain condition of running signal successfully set
    */
    std::condition_variable cvRun_;

    /**
     * @brief Flag for runnning the platform
     * 
     * The flag is set true by the @sa run function 
     * It is set false in some parts of the code to maintain accuracy 
    */
    std::atomic<bool> running_;
    
    /**
     * @brief Mutex for use in coordination with conditional variable in execution function
     * 
     * Used to lock and unlock while waiting for the condition variable @sa cvRun_ to be notified 
     * by the @sa run function
    */
    std::mutex mtxRunning_;

    /**
     * @brief Type of platform 
     * 
     * Enum type of the type of platform the controller is
     * Can take either value ACKERMAN or QUADCOPTER
    */
    pfms::PlatformType platformType_;

    unsigned int marker_counter_;
    
    ros::Rate rateLimit_;
    
    sensor_msgs::LaserScan laserScan_;

    // --------------// HELPER FUNCTION // --------------------------------------------------------//

    /**
     * @brief Euclidean distance
     * 
     * Calculate the distance between 2 points given their coordinates
     * 
     * @return the distance between 2 points 
    */
    double euclideanDistance(double x1, double y1, double x2, double y2);

    /**
     * @brief Normalize angle
     * 
     * Normalize the input angle to the range of -π (M_PI) to π.
     * 
     * @return the normalized angle
    */
    double normalizeAngle(double angle);


private:

    /**
     * Callback function for odometry
     */
    void odoCallback (const nav_msgs::Odometry::ConstPtr& msg);

    /**
     * Callback function for laser scan
     */
    void laserCallback (const sensor_msgs::LaserScan::ConstPtr& msg);


    // /**
    //  * @brief Controller odometry
    // */
    // nav_msgs::Odometry odo_;//!< The current pose of platform
    nav_msgs::Odometry odo_;
    
    /**
     * @brief Laser scan data
    */


    /**
     * @brief Mutex for shared variable for odometry information
     * 
     * Used to protect the odo_ member variable which is shared by the main and an execution thread
    */
    std::mutex mtxOdo_;     /*!< Mutex to lock robotPose_ */


};

#endif