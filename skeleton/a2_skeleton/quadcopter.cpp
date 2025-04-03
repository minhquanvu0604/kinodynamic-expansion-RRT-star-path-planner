#include <cmath>
#include <iostream>

#include "quadcopter.h"
#include "pipes.h"
// #include "pfms_types.h"


Quadcopter::Quadcopter(){
    platformType_ = pfms::PlatformType::QUADCOPTER;

    // Initialize a new thread and push it in the vector of threads
    threads_.push_back(std::thread(&Quadcopter::executeQuadcopter,this));
}


/**
 * @brief Setter for goals
 * 
 * Call @sa checkOriginToDestination to update the member variable regarding distance to goal and time to goal
 * 
 * @param goals The vector of points as goals
 * @return Whether all goals are reachable, in order supplied. Returns false if any goal is unreachable
*/
bool Quadcopter::setGoals(std::vector<pfms::geometry_msgs::Point> goals){
    
    goals_ = goals;

    // Dummy variable
    pfms::nav_msgs::Odometry estimatedGoalPose;

    // Update distanceToGoal_, timeToGoal for quadcopterReachGoal test only
    // Not the main purpose of checkOriginToDestination
    auto odo = this->Controller::getOdometry();
    bool OK = checkOriginToDestination(odo, goals_.at(0), distanceToGoal_, timeToGoal_, estimatedGoalPose);

    return OK;
}


/**
 * @brief Checks whether the platform can travel between origin and destination
 * 
 * The point variable is assigned to be the goal point. The endpoint yaw is exactly the same as the starting yaw.
 * This is because we assume there is no rotation of the Quadcopter throughout its motion.
 * 
 * The distance is a simple linear distance between two points in the cartesian coordinate
 * 
 * The time estimated to travel to the goal is evaluated based on the recommened velocity which is currently 0.4 [m/s]
 *
 * The function always return true because the the Quadcopter can travel to any position 
 *  
 * @param[in] origin The origin pose, specified as odometry for the platform
 * @param[in] destination The destination point for the platform
 * @param[in|out] distance The distance [m] the platform will need to travel between origin and destination. If destination unreachable distance = -1
 * @param[in|out] time The time [s] the platform will need to travel between origin and destination, If destination unreachable time = -1
 * @param[in|out] estimatedGoalPose The estimated goal pose when reaching goal. The point variable is assigned to be the goal point. The endpoint yaw requires further calculation
 * @return bool indicating the platform can reach the destination from origin supplied
*/
bool Quadcopter::checkOriginToDestination(pfms::nav_msgs::Odometry origin,
                                        pfms::geometry_msgs::Point goal,
                                        double& distance,
                                        double& time,
                                        pfms::nav_msgs::Odometry& estimatedGoalPose){
    
    // Update estimated goal pose                                        
    estimatedGoalPose.position.x = goal.x;
    estimatedGoalPose.position.y = goal.y;
    estimatedGoalPose.yaw = origin.yaw;

    // Update distance to goal
    distance = euclideanDistance(origin.position.x, origin.position.y, goal.x, goal.y);

    // Update time to goal
    time = distance / recommendedVelocity;
    
    // Always posible for a quadcopter to reach any goal
    return true;
    }


/**
 * @brief Execute the Ackerman to reach goals
 * 
 * This function is ran in a separate thread to facilitate concurrency of execution
 * The function starts to execute when the new object is constructed. It then waits in the statement with wait() function 
 * of the conditional variable object. When all the necessary conditions are met in the @sa run function, the conditional variable 
 * sends notification to lock the mtxRunning_ and progress with execution. Then the Quadcopter starts to run, hence the platform status
 * now turns to RUNNING
 * 
 * The start time is recorded from the start of the function and end time at the end to count the time duration of execution through all the goals
 * 
 * The running comprises two while loop for each goal, with one loop for the runnning and the other for braking
 * 
 * For the quadcopter, the angle between its heading and the goal is constantly updated. Hence, it recalculates to move towards the goal at every 
 * time step
 * 
 * The second loop for braking is simply sending a command with all zeros when the Quadcopter is tolerance_ liner distance away from the goal
 * 
 * In the two loops the distance travelled by the Quadcopter is recorded. It is the sum of the distances of the current position from the position 
 * in the last time step. Additionally, time elapsed is also recorded from the start to end of the function
*/
void Quadcopter::executeQuadcopter(){

    // Protecting running_ variable
    std::unique_lock<std::mutex> lck(mtxRunning_);

    // Wait for running signal
    cvRun_.wait(lck, [this]{return running_.load();});
    lck.unlock();

    // Let's take off here, we send the status to the platform
    auto takeoff = pfms::PlatformStatus::TAKEOFF;
    pipesPtr_->send(takeoff);

    // Dummy variable
    pfms::nav_msgs::Odometry dummyEstimatedGoalPose;

    // Start counting time
    auto startTime = std::chrono::high_resolution_clock::now();

    // Start counting past distance, this variable is updated at the completion of reaching each goal
    double distancePastGoals = 0;

    // Sequence
    unsigned long i = 0;

    // Go through all the goals
    for (int g = 0; g < goals_.size(); g++){

        // Update distanceToGoal_ for calculating distancePastGoals
        auto odoStart = getOdometry();
        checkOriginToDestination(odoStart, goals_.at(g), distanceToGoal_, timeToGoal_, dummyEstimatedGoalPose);

        // Run the quadcopter   
        while (running_){

            // Perform calculations
            auto currentPose = Controller::getOdometry();

            // Convert the goal to local frame
            auto localGoal = globalToLocalFrame(goals_.at(g), currentPose.position, currentPose.yaw);

            // Compute velocity components on x and y
            double angleToGoal = atan2(localGoal.y, localGoal.x);
        
            // Calculate the velocity in x and y axis
            double velo_x = recommendedVelocity * cos(angleToGoal);
            double velo_y = recommendedVelocity * sin(angleToGoal);

            // Initializing and sending commands
            pfms::commands::UAV uav {i, 0, velo_y, 0, velo_x};
            pipesPtr_->send(uav);
            i++;

            // We wait for a short time, just to enable remaining of system to respond, recommended to wait
            std::this_thread::sleep_for(std::chrono::milliseconds(10));


            // Calculate travelled path to between the lastest past goal and the next goal, update total distance travelled
            // The travelledPath is updated every time step
            double travelledPath = 0;
            double dummyDistance = 0;
            checkOriginToDestination(odoStart, currentPose.position, travelledPath, dummyDistance, dummyEstimatedGoalPose);

            std::unique_lock<std::mutex> lckDistanceTravelled(mtxDistanceTravelled_);
            distanceTravelled_ = distancePastGoals + travelledPath;
            lckDistanceTravelled.unlock();

            // Stop running when tolerance away from the goal
            double instantDistanceToGoal = euclideanDistance(currentPose.position.x, currentPose.position.y, goals_.at(g).x, goals_.at(g).y);

            // Break the running loop if it gets close to the goal by tolerance_
            if(instantDistanceToGoal < tolerance_) break;
        }  
        
        // Stop the quadcopter
        while (running_){
            pfms::commands::UAV uav {i, 0, 0, 0, 0};
            pipesPtr_->send(uav);
            i++;

            // We wait for a short time, just to enable remaining of system to respond, recommended to wait
            std::this_thread::sleep_for(std::chrono::milliseconds(10));

            // Update odo_
            auto currentPose = Controller::getOdometry();

            // Compute velocity
            double velocity = std::pow(std::pow(currentPose.linear.x,2)+ std::pow(currentPose.linear.y,2),0.5);
            if(velocity < 0.1) break;
        }

        distancePastGoals += distanceToGoal_;
    } 

    // Count time
    auto endTime = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsedTime = endTime - startTime;
    
    std::unique_lock<std::mutex> lckTimeTravelled(mtxTimeTravelled_);
    timeTravelled_ = elapsedTime.count();
    lckTimeTravelled.unlock();

    // Update status
    platformStatus_ = pfms::PlatformStatus::IDLE;
}