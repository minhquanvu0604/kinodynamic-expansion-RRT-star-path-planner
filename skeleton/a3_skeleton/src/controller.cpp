#include "controller.h"


/**
 * @brief Default destructor
 * 
 * Shared functionality/base class for platform controllers
 * Default desstructors set necessary varibles to terminate the execution of platforms
 * (the running_ boolean is the condition for the while looping which sends commands to platforms)
 * and join all the threads
 */
Controller::Controller() : running_{false}, marker_counter_{0}, advanced_{true}, tolerance_{0.5}, 
                            rateLimit_{5}, id_{0} {

    subOdom_ = nh_.subscribe("/ugv_odom/", 1000, &Controller::odoCallback,this);
    subLaser_ = nh_.subscribe("/orange/laser/scan", 1000, &Controller::laserCallback,this);

    tolerance_ = 0.5;//We set tolerance to be default of 0.5

    // //We set the internal variables of time/distance for goal to zero
    // goal_.time=0;
    // goal_.distance=0;
}

Controller::~Controller(){
    running_ = false;
    
    // Join threads
    for(auto & t: threads_)
        t.join();
}

void Controller::odoCallback (const nav_msgs::Odometry::ConstPtr& msg){
    std::unique_lock<std::mutex> lck (mtxOdo_);
    odo_ = *msg; 
}

void Controller::laserCallback (const sensor_msgs::LaserScan::ConstPtr& msg){
    std::unique_lock<std::mutex> lck (mtxLaser_);
    laserScan_ = *msg;
}

/**
 * Run controller in reaching goals - non blocking call
 */
void Controller::run(void){
    std::unique_lock<std::mutex> lck(mtxRunning_);
    running_ = true;
    cvRun_.notify_all();
}

/**
 * Getter for pltform type
 * @return PlatformType
 */
pfms::PlatformType Controller::getPlatformType(void){
    return platformType_;
}

/**
 * Set tolerance when reaching goal
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
 * returns current odometry information
 * @return odometry - current odometry
 */
nav_msgs::Odometry Controller::getOdometry(void) {
    std::unique_lock<std::mutex> lckTimeTravelled(mtxOdo_);
    return odo_;
}


// --------------// HELPER FUNCTION // --------------------------------------------------------//

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




