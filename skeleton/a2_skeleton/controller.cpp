#include "controller.h"
#include "pipes.h"

/**
 * @brief Default constructor
 * 
 * Default constructors should set all sensor attributes and necessary
 * member variables to a default value. It involves setting the, tolerance to 0.5, the status to idle,
 * the time travelled to 0 and the time to next goal to 0 
 * It also initilizes a pipeline to gazebo by creating a smart pointer to a user-defined-type Pipes object
*/
Controller::Controller() : tolerance_{0.5}, 
                        platformStatus_{pfms::PlatformStatus::IDLE}, 
                        running_{false}, distanceTravelled_{0}, timeTravelled_{0}, timeToGoal_{0}{

    // Initialize smart ptr to Pipes object
    pipesPtr_ = std::make_shared<Pipes>();
}


/**
 * @brief Default destructor
 * 
 * Default desstructors set necessary varibles to terminate the execution of platforms
 * (the running_ boolean is the condition for the while looping which sends commands to platforms)
 * and join all the threads
*/
Controller::~Controller(){
    running_ = false;
    
    // Join threads
    for(auto & t: threads_)
        t.join();
}


/**
 * @brief Run controller in reaching goals 
 * 
 * Non blocking call to start the running of platforms by
 * setting a flag to start the execution function 
 * After that the conditional variable executes a function to send notification to start 
 * the running in the @sa executeAckerman or @sa executeQuadcopte, which are member functions of the 
 * two derived classes of the corresponding platforms and are ran in separate threads
*/
void Controller::run(){

    std::unique_lock<std::mutex> lck(mtxRunning_);
    running_ = true;
    cvRun_.notify_one();

    platformStatus_ = pfms::PlatformStatus::RUNNING;
}

/**
 * @brief Returns platform status
 * 
 * Indicating if it is executing a series of goals or idle - waiting for goals
 * The return value is either pfms::PlatformStatus::RUNNING or pfms::PlatformStatus::IDLE
 * No further execution
 * 
 * @return platform status, whether it is idle or running
*/
pfms::PlatformStatus Controller::status(void){
    return platformStatus_; 
}


/**
 * @brief Getter for platform type of the controller
 * 
 * The type of platform available is either pfms::PlatformType::ACKERMAN or pfms::PlatformType::QUADCOPTER
 * 
 * @return PlatformType
*/
pfms::PlatformType Controller::getPlatformType(void){
    return platformType_;
}


/**
 * @brief Getter for distance to be travelled to reach current goal
 * 
 * Returns the member variable, no further execution
 * 
 * @return distance to be travlled to reach current goal [m]
*/
double Controller::distanceToGoal(void){
    return distanceToGoal_;
}


/**
 * @brief Getter for time to reach current goal
 * 
 * Returns the member variable, no further execution
 * 
 * @return time to travel to current goal [s]
*/
double Controller::timeToGoal(void){
    return timeToGoal_;
}


/**
 * @brief Set tolerance when reaching goal
 * 
 * Setting of the tolerance which returns true only when it is a non-negative number
 * 
 * @return tolerance accepted [m]
*/
bool Controller::setTolerance(double tolerance){
    if (tolerance >= 0) {
        tolerance_ = tolerance;
        return true;
    }
    else {
        tolerance_ = std::abs(tolerance);
        return false;
    }
}


/**
 * @brief Returns distance travelled by platform
 * 
 * Returns the member variable, no further execution
 * 
 * @return total distance travelled since execution @sa run called with goals supplied
*/
double Controller::distanceTravelled(void){
    std::unique_lock<std::mutex> lckDistanceTravelled(mtxDistanceTravelled_);
    return distanceTravelled_;
}


/**
 * @brief Returns total time in motion by platform
 * 
 * Returns the member variable, no further execution
 * 
 * @return total time in motion since execution @sa run called with goals supplied
*/
double Controller::timeTravelled(void){
    std::unique_lock<std::mutex> lckTimeTravelled(mtxTimeTravelled_);
    return timeTravelled_;
}


/**
 * @brief Return current odometry information
 * 
 * This function takes the reference of odo_
 * Protected with mutex to be shared by the main and excution thread
 * 
 * @return odometry - current odometry
*/
pfms::nav_msgs::Odometry Controller::getOdometry(void){

    std::unique_lock<std::mutex> lckTimeTravelled(mtxOdo_);

    pipesPtr_->read(odo_, platformType_);

    return odo_;
}


// --------------// HELPER FUNCTION // ----------------------------------------------------------------------//

/**
 * @brief Convert to local frame 
 * 
 * Convert the goal point from global frame of reference to local frame of robot
 * Private function as it is only called by @sa checkOriginToDestination function
*/
pfms::geometry_msgs::Point Controller::globalToLocalFrame(pfms::geometry_msgs::Point goal, pfms::geometry_msgs::Point robotPosition, double robotYaw) {
    // Calculate the relative position of the goal in the global frame
    double dx = goal.x - robotPosition.x;
    double dy = goal.y - robotPosition.y;

    // Convert the relative position to the robot frame using the robot's yaw angle
    double goalRobotFrame_x = dx * cos(robotYaw) + dy * sin(robotYaw);
    double goalRobotFrame_y = -dx * sin(robotYaw) + dy * cos(robotYaw);

    pfms::geometry_msgs::Point goalRobotFrame = {goalRobotFrame_x, goalRobotFrame_y};
    return goalRobotFrame;
}


/**
 * @brief Euclidean distance
 * 
 * Calculate the distance between 2 points given their coordinates
 * Private function as it is only called by other member functions
 * 
 * @return the distance between 2 points 
*/
double Controller::euclideanDistance(double x1, double y1, double x2, double y2) {
    return std::sqrt(std::pow(x2 - x1, 2) + std::pow(y2 - y1, 2));
}


/**
 * @brief Normalize angle
 * 
 * Normalize the input angle to the range of -π (M_PI) to π
 * Private function as it is only called by other member functions
 * 
 * @return the normalized angle
*/
double Controller::normalizeAngle(double angle)
{
    angle = std::fmod(angle + M_PI, 2 * M_PI);
    if (angle < 0) {
        angle += 2 * M_PI;
    }
    return angle - M_PI;
}

// -------------------------------------------------------------------------------------------------------------//