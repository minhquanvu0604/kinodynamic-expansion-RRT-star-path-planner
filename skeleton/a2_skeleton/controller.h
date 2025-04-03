#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "controllerinterface.h"
#include "pipes.h"

//See controllerinterface.h for more information
class Controller: public ControllerInterface
{
public:
    
    /**
     * @brief Default constructor
     * 
     * Default constructors should set all sensor attributes and necessary
     * member variables to a default value and initilize a pipeline to gazebo
    */
    Controller();


    /**
     * @brief Default destructor
     * 
     * Default desstructors set necessary varibles to terminate the execution of platforms
     * and join all the threads
    */
    ~Controller();


    /**
     * @brief Run controller in reaching goals 
     * 
     * Non blocking call to start the running of platforms by
     * setting a flag to start the execution function 
    */
    virtual void run(void);


    /**
     * @brief Returns platform status
     * 
     * Indicating if it is executing a series of goals or idle - waiting for goals
     * No further execution
     * 
     * @return platform status, whether it is idle or running
    */
    virtual pfms::PlatformStatus status(void);


    /**
     * @brief Setter for goals
     * 
     * Set a series of goals as point cartesian coordination into a vector 
     * 
     * @param goals
     * @return all goal reachable, in order supplied    
    */
    virtual bool setGoals(std::vector<pfms::geometry_msgs::Point> goals) = 0;


    /**
     * @brief Checks whether the platform can travel between origin and destination
     * 
     * The function also takes reference and modifies necessary variables to specific use
     * 
     * @param[in] origin The origin pose, specified as odometry for the platform
     * @param[in] destination The destination point for the platform
     * @param[in|out] distance The distance [m] the platform will need to travel between origin and destination. If destination unreachable distance = -1
     * @param[in|out] time The time [s] the platform will need to travel between origin and destination, If destination unreachable time = -1
     * @param[in|out] estimatedGoalPose The estimated goal pose when reaching goal
     * @return bool indicating the platform can reach the destination from origin supplied
    */
    virtual bool checkOriginToDestination(pfms::nav_msgs::Odometry origin,
                                          pfms::geometry_msgs::Point goal,
                                          double& distance,
                                          double& time,
                                          pfms::nav_msgs::Odometry& estimatedGoalPose) = 0;


    /**
     * @brief Getter for platform type of the controller
     * 
     * The types of platform available are ackerman and quadcopter 
     * 
     * @return PlatformType
    */
    pfms::PlatformType getPlatformType(void);


    /**
     * @brief Getter for distance to be travelled to reach current goal
     * 
     * @return distance to be travlled to reach current goal [m]
    */
    virtual double distanceToGoal(void);


    /**
     * @brief Getter for time to reach current goal
     * 
     * @return time to travel to current goal [s]
    */
    virtual double timeToGoal(void);


    /**
     * @brief Set tolerance when reaching goal
     * 
     * Setting of the tolerance which returns true only when it is a non-negative number
     * 
     * @return tolerance accepted [m]
    */
    bool setTolerance(double tolerance);


    /**
     * @brief Returns distance travelled by platform
     * 
     * @return total distance travelled since execution @sa run called with goals supplied
    */
    virtual double distanceTravelled(void);


    /**
     * @brief Returns total time in motion by platform
     * 
     * @return total time in motion since execution @sa run called with goals supplied
    */
    virtual double timeTravelled(void);


    /**
     * @brief Return current odometry information
     * 
     * This function takes the reference of odo_
     * Protected with mutex to be shared by the main and excution thread
     * 
     * @return odometry - current odometry
    */
    virtual pfms::nav_msgs::Odometry getOdometry(void);


protected:

    /**
     * @brief Pointer to a Pipes object
     * 
     * This variable facilitates the communication between ROS and student program
     * Interfacing through this pointer abstracts the underlying work of ROS
    */
    std::shared_ptr<Pipes> pipesPtr_;

    /**
     * @brief Controller odometry
     * 
     * This variable contains the instantaneous odometry of the platform which can be read and sent through the Pipe
    */
    pfms::nav_msgs::Odometry odo_; 
  
    /**
     * @brief Type of platform 
     * 
     * Enum type of the type of platform the controller is
     * Can take either value ACKERMAN or QUADCOPTER
    */
    pfms::PlatformType platformType_;

    /**
     * @brief Tolerance
     * 
     * The tolerance value of the straight distance from the platform to the goal
     * Used to judge whether the platforms have accomplished reaching the goals
    */
    double tolerance_;

    /**
     * @brief Distance to goal
     * 
     * The distance to goal point, updated in @sa reachGoal where new goal is set
    */
    double distanceToGoal_;
  
    /**
     * @brief Time to goal
     * 
     * The estimated time to reach the upcoming goal point, updated in @sa reachGoal where new goal is set
    */
    double timeToGoal_;

    /**
     * @brief Distance travelled
     * 
     * The distance the platform has travelled from program start
     * Updated at every time step
    */
    double distanceTravelled_;

    /**
     * @brief Total time that platform has been moving
     * 
     * Updated in @sa reachGoal, calculated as the sum of time intervals of executions of @sa reachGoal
    */
    double timeTravelled_;

    /**
     * @brief A private copy of goals
     * 
     * A vector of Point objects
    */
    std::vector<pfms::geometry_msgs::Point> goals_; 

    /**
     * @brief Platform status
     * 
     * Status of the platform, either IDLE, RUNNING, TAKEOFF or LANDING
     * TAKEOFF and LANDING are for the UAV only
    */
    pfms::PlatformStatus platformStatus_;

    /**
     * @brief Vector of thread objects
     * 
     * Contains the threads which execute different platforms
     * Each element is corresponding to a platform
    */
    std::vector<std::thread> threads_;

    /**
     * @brief Mutex for shared variable for odometry information
     * 
     * Used to protect the odo_ member variable which is shared by the main and an execution thread
    */
    std::mutex mtxOdo_;

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
     * @brief Conditional variable  
     * 
     * Used to synchronize the running of platforms after certain condition of running signal successfully set
    */
    std::condition_variable cvRun_;

    /**
     * @brief Mutex for shared variable for the time the platform has travelled  
     * 
     * It is necessary because the @sa timeTravelled_ member variable is read and modified continously
     * by both the main thread and the running execution thread at every time step
    */
    std::mutex mtxTimeTravelled_;

    /**
     * @brief Mutex for shared variable for the distance the platform has travelled  
     * 
     * It is necessary because the @sa distanceTravelled_ member variable is read and modified continously
     * by both the main thread and the running execution thread at every time step
    */
    std::mutex mtxDistanceTravelled_;


    // --------------// HELPER FUNCTION // --------------------------------------------------------//

    /**
     * @brief Convert to local frame 
     * 
     * Convert the goal point from global frame of reference to local frame of robot
    */
    pfms::geometry_msgs::Point globalToLocalFrame(pfms::geometry_msgs::Point goal, pfms::geometry_msgs::Point robotPosition, double robotYaw);

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
};


#endif // CONTROLLER_H
