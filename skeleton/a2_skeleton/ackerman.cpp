#include <cmath>
#include <iostream>

#include "ackerman.h"
#include "pipes.h"


/**
 * @brief Default constructor
 * 
 * Default constructor should set all sensor attributes to a default value, here the default steering is 0
 * and the platform type to indicate ackerman
 * It also initialize a new thread object to execute the running of Ackerman in a new thread
*/
Ackerman::Ackerman() : steering_{0} {
    platformType_ = pfms::PlatformType::ACKERMAN;

    // Initialize a new thread and push it in the vector of threads
    threads_.push_back(std::thread(&Ackerman::executeAckerman,this));
}


/**
 * @brief Setter for goals
 * 
 * Call @sa checkOriginToDestination to update the member variable regarding distance to goal and time to goal
 * 
 * @param goals The vector of points as goals
 * @return Whether all goals are reachable, in order supplied. Returns false if any goal is unreachable
*/
bool Ackerman::setGoals(std::vector<pfms::geometry_msgs::Point> goals){

    pfms::nav_msgs::Odometry estimatedGoalPose = this->Controller::getOdometry();    

    // For the checkOriginToDestination unit test, have to update distanceToGoal_ and timeToGoal_
    for (int g = 0; g < goals.size(); g++){
        bool OK = checkOriginToDestination(estimatedGoalPose, goals.at(g), distanceToGoal_, timeToGoal_, estimatedGoalPose);
        if (!OK) return false;
    }

    goals_ = goals;
    return true;
}


/**
 * @brief Checks whether the platform can travel between origin and destination
 * 
 * The funciton first calls the @sa globalToLocalFrame function to convert the goal point from the global frame to the local frame of the audi
 * Then the yaw angle (alpha) between the heading of the audi and the goal point can be calculated
 * The sweeping angle is the angle that the audi sweeps from the starting to the end point, with the apex being the center of rotation. It can be
 * calculated to be 2*alpha according to the ackerman formula
 * Then, calculation of turning radius facilitates the calculation of the distance to goal   
 * If we want to update the steering_ member variable, we can set the updateSteering_ boolean variable to true before the call to checkOriginToDestination
 * (is was set true in executeAckerman function to evaluate the steering command going to the upcoming goal)
 * 
 * @param[in] origin The origin pose, specified as odometry for the platform
 * @param[in] destination The destination point for the platform
 * @param[in|out] distance The distance [m] the platform will need to travel between origin and destination. If destination unreachable distance = -1
 * @param[in|out] time The time [s] the platform will need to travel between origin and destination, If destination unreachable time = -1
 * @param[in|out] estimatedGoalPose The estimated goal pose when reaching goal. The point variable is assigned to be the goal point. The endpoint yaw requires further calculation
 * @return bool indicating the platform can reach the destination from origin supplied
*/
bool Ackerman::checkOriginToDestination(pfms::nav_msgs::Odometry origin,
                                        pfms::geometry_msgs::Point goal, 
                                        double& distance,
                                        double& time,
                                        pfms::nav_msgs::Odometry& estimatedGoalPose){

    // Convert the coordinate of goal point from global frame to local frame    
    pfms::geometry_msgs::Point carOrigin = {origin.position.x, origin.position.y};
    pfms::geometry_msgs::Point globalGoal = {goal.x, goal.y};

    pfms::geometry_msgs::Point localGoal = globalToLocalFrame(globalGoal, carOrigin, origin.yaw );                    


    // Calculate the straight line distance from origin to target
    double linearDistanceToGoal = std::hypot(localGoal.x,localGoal.y);

    // Calculate alpha, this alpha ranges from -pi to pi
    double alpha = atan2(localGoal.y, localGoal.x);

    // Find and correct the sweeping angle, which depends on the quadrant of goal point in local frame (while alpha does not) 
    double sweepingAngle = 2 * alpha; 

    // Calculate the turning radius R
    double turningRadius = linearDistanceToGoal / (2 * sin(std::abs(alpha)));

    // Update distancce and time to goal
    distance = std::abs(sweepingAngle) * turningRadius;
    time  = distance / CONSTANT_SPEED;

    // Update estimated goal pose
    estimatedGoalPose.position.x = goal.x;
    estimatedGoalPose.position.y = goal.y;

    // Normalize the input angle to the range of -π (M_PI) to π
    estimatedGoalPose.yaw = Controller::normalizeAngle(origin.yaw + sweepingAngle);
 
    // Update steering 
    double steeringAngle = atan2(WHEELBASE * 2 * sin(alpha),linearDistanceToGoal);
    if (updateSteering_){
        steering_ = steeringAngle * STEERING_RATIO;
    }

    // Check if less than the max steering angle allowed
    bool possible = std::abs(steeringAngle) < MAX_STEER_ANGLE;

    if (updateTimeDist_)
        if (!possible){
            time = -1;
            distance = -1;
        }

    return possible;    
}            


/**
 * @brief Execute the Ackerman to reach goals
 * 
 * This function is ran in a separate thread to facilitate concurrency of execution
 * The function starts to execute when the new object is constructed. It then waits in the statement with wait() function 
 * of the conditional variable object. When all the necessary conditions are met in the @sa run function, the conditional variable 
 * sends notification to lock the mtxRunning_ and progress with execution. Then the Ackerman starts to run, hence the platform status
 * now turns to RUNNING
 * 
 * The start time is recorded from the start of the function and end time at the end to count the time duration of execution through all the goals
 * 
 * The running comprises two while loop for each goal, with one loop for the runnning and the other for braking
 * 
 * The workflow of the first loop is basically feeding the fixed steering for the current goal, and the throttle is constant to be 0.1 throughout. Additionally, 
 * the linear distance to goal is continuously updated at every time step to figure out how close the audi has got the goal. If that 
 * distance gets smaller than the brakeThreshold, the running loop ends. The brakeThreshold is calculated by a self-developed formula utilizing linear regression, which increases 
 * as the speed increases
 * 
 * The second loop handles the brake. It is provided by a different formula which increase the brake torque as the audi gets closer to the goal
 * The brake torque value starts from the variable indicating the min brake torque and gets to the max value when it is tolerance distance away 
 * from the goal. The second loop exits when the velocity of the audi becomes lower than the maxSpeedToProgressToNextGoal variable
 * 
 * In the two loops the distance travelled by the audi is recorded. It is the distance to goal of the whole paths between goals that have been
 * gone through, and the partial distance of the current position from the last goal
*/
void Ackerman::executeAckerman(){

    // Protecting running_ variable
    std::unique_lock<std::mutex> lckRunning(mtxRunning_);

    // Wait for running signal
    cvRun_.wait(lckRunning, [this]{return running_.load();});
    lckRunning.unlock();

    // Constants for control tuning, empirical
    // Progress to the next goal when this speed lowered to this value 
    double maxSpeedToProgressToNextGoal = 0.05;    
    // Empirical value of the minimum brake signal fed to the Pipe
    double minBrake = 5740;
    double brake = 0;
    double brakeThreshold = 0.3; // Default value 

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

        // Get current odometry
        pfms::nav_msgs::Odometry currentPose = Controller::getOdometry();

        // Update steering_ towards goal g
        // The intended use of checkDistanceToDestination, so we set updateSteering_ and updateTimeDist_ to true 
        updateSteering_ = true;
        checkOriginToDestination(currentPose, goals_.at(g), distanceToGoal_, timeToGoal_, dummyEstimatedGoalPose);
        // Set it false right after calling
        updateSteering_ = false;

        // The steering_ value coming after the intended use of checkDistanceToDestination is the correct one
        double inputSteering = steering_;

        // Run the ackerman
        while (running_){

            // Update odo_
            auto odo = Controller::getOdometry();

            // Calculate travelled path to between the lastest past goal and the next goal, update total distance travelled
            // The travelledPath is updated every time step
            double travelledPath = 0;
            double dummyDistance = 0;
            checkOriginToDestination(currentPose, odo.position, travelledPath, dummyDistance, dummyEstimatedGoalPose);
            
            std::unique_lock<std::mutex> lckDistanceTravelled(mtxDistanceTravelled_);
            // Update distanceTravelled_ when in the middle of travelling to goal
            distanceTravelled_ = distancePastGoals + travelledPath;
            lckDistanceTravelled.unlock();

            // Initializing and sending commands
            pfms::commands::UGV ugv {i, 0, inputSteering, fixThrottle};      
            pipesPtr_->send(ugv);
            i++;

            // We wait for a short time, just to enable remaining of system to respond, recommended to wait
            std::this_thread::sleep_for(std::chrono::milliseconds(10));


            // brakeThreshold: the minimum distance between the platform and goal to start applying brake
            // Tuning breakThreshold : plot the successful breakThreshold against the speed when braking starts 
            // Using linear regression found y=0.28x-0.43
            brakeThreshold = std::abs(0.28 * (std::abs(odo.linear.x)+std::abs(odo.linear.y)) - 0.48); 
            if (brakeThreshold < tolerance_) brakeThreshold = tolerance_; 

            // Stop running when tolerance away from the goal
            double instantDistanceToGoal = euclideanDistance(odo.position.x, odo.position.y, goals_.at(g).x, goals_.at(g).y);

            // Break the running loop if it gets close to the goal by breakThreashold
            if(instantDistanceToGoal < brakeThreshold) break;
        }  
        
        // Stop the ackerman
        while (running_){

            // Update odo_
            auto odo = Controller::getOdometry();

            // The break toruqe fed to the Pipe
            // The brake value is larger as the platforms approaches the goal, starting from minBrake to max brake torque
            double instantDistanceToGoal = euclideanDistance(odo.position.x, odo.position.y, goals_.at(g).x, goals_.at(g).y);
            brake = ((MAX_BRAKE_TORQUE - minBrake) / brakeThreshold) * (brakeThreshold - instantDistanceToGoal) + minBrake;

            // When the platform is tolerance_ value away from the goal, apply the max brake torque
            if (instantDistanceToGoal < tolerance_) brake = MAX_BRAKE_TORQUE;

            // Sending commands
            pfms::commands::UGV ugv {i, brake, inputSteering, 0};
            pipesPtr_->send(ugv);
            i++;

            // We wait for a short time, just to enable remaining of system to respond, recommended to wait
            std::this_thread::sleep_for(std::chrono::milliseconds(10));

            // Compute velocity
            double velocity = std::pow(std::pow(odo.linear.x,2)+ std::pow(odo.linear.y,2),0.5);

            // Progress to next goal
            if(std::abs(velocity) < maxSpeedToProgressToNextGoal) break;
        }

        // Update distanceTravelled_ at the end of reaching goals
        distancePastGoals += distanceToGoal_;
        distanceTravelled_ = distancePastGoals;
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
    
