#include <cmath>
// #include <limits>
#include <iostream>

#include "ackerman.h"
#include "pipes.h"
 

Ackerman::Ackerman(){
    // Initialize some variables 
    platformType_ = pfms::PlatformType::ACKERMAN;
    distanceTravelled_ = 0;
    timeInMotion_ = 0;
}
/*

    // Start counting time
    auto startTime = std::chrono::high_resolution_clock::now();

    // Fetch odometry
    auto odo = this->getOdometry();

    // Constants for control tuning, empirical
    // Progress to the next goal when this speed lowered to this value 
    double maxSpeedToProgressToNextGoal = 0.1;    
    // Empirical value of the minimum brake signal fed to the Pipe
    double minBrake = 5740;

    double brake;
    double brakeThreshold = 0.3; // Default value 

    // Initialize step count
    unsigned long i = 0;

    // Start moving towards goal
    while(true) {

        // Sending commands
        pfms::commands::UGV ugv {
                    i,
                    0,
                    steering_,
                    fixThrottle
                 };
        pipesPtr->send(ugv);

        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        // Fetch odometry
        odo = this->getOdometry();

        // brakeThreshold: the minimum distance between the platform and goal to start applying brake
        // Tuning breakThreshold : plot the successful breakThreshold against the speed when braking starts 
        // Using linear regression found y=0.28x-0.43
        brakeThreshold = std::abs(0.28 * (std::abs(odo.linear.x)+std::abs(odo.linear.y)) - 0.48); 

        if (instantLinearDistanceToGoal_ < brakeThreshold) break;

        // Increase step count
        i++;  
    } 

    // Reset sequence
    i = 0;

    // Start slowing down
    while(true){
        
        // The break toruqe fed to the Pipe
        // The brake value is larger as the platforms approaches the goal, starting from minBrake to max brake torque
        brake = ((MAX_BRAKE_TORQUE - minBrake) / brakeThreshold) * (brakeThreshold - instantLinearDistanceToGoal_) + minBrake;

        // When the platform is tolerance_ value away from the goal, apply the max brake torque
        if (instantLinearDistanceToGoal_ < tolerance_) brake = MAX_BRAKE_TORQUE;

        // Sending commands        
        pfms::commands::UGV ugv {
                    i,
                    brake, 
                    steering_,
                    0
                };
        pipesPtr->send(ugv);
        
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        // Fetch Odometry
        odo = this->getOdometry();

        // Progress to the next goal
        if (std::abs(odo_.linear.x) < maxSpeedToProgressToNextGoal
                && std::abs(odo_.linear.y) < maxSpeedToProgressToNextGoal)
                break;
            
        // Increase step count
        i++;  
    }

    // Count time
    auto endTime = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsedTime = endTime - startTime;
    timeInMotion_ += elapsedTime.count();

    if (instantLinearDistanceToGoal_ < tolerance_) return true;
    else return false;
}

bool Ackerman::setGoal(pfms::geometry_msgs::Point commandGoal){
    // Update global goal
    globalGoal_ = commandGoal;

    // Fetch odometry
    auto odo = getOdometry();

    bool OK = checkOriginToDestination(odo, commandGoal, distanceToGoal_, timeToGoal_, estimatedGoalPose_);

    return OK;
}

bool Ackerman::checkOriginToDestination(pfms::nav_msgs::Odometry origin,
                                        pfms::geometry_msgs::Point goal, 
                                        double& distance,
                                        double& time,
                                        pfms::nav_msgs::Odometry& estimatedGoalPose){
        
    // // Convert the coordinate of goal point from global frame to local frame    
    pfms::geometry_msgs::Point carOrigin = {origin.position.x, origin.position.y};
    pfms::geometry_msgs::Point globalGoal = {goal.x, goal.y};

    pfms::geometry_msgs::Point localGoal = globalToLocalFrame(globalGoal, carOrigin, origin.yaw );

    // Update loca goal internal variable
    localGoal_.x = localGoal.x;
    localGoal_.y = localGoal.y;

    // Go on a straight line
    if (!localGoal_.y) {
        distance = localGoal_.x;
        time = distance / CONSTANT_SPEED;
        steering_ = 0;
        return true;
    }
    
    // Calculate the straight line distance from origin to target
    double linearDistanceToGoal = std::hypot(localGoal_.x,localGoal_.y);

    // Calculate alpha, this alpha ranges from -pi to pi
    double alpha = atan2(localGoal_.y, localGoal_.x);

    // Change alpha to be the shortest angle, i.e ranges from -pi/2 to pi/2
    if (alpha > M_PI /2) 
        alpha = M_PI - alpha;
    else if (alpha < -M_PI /2 ) 
        alpha = -(M_PI + alpha);

    // Find and correct the sweeping angle, which depends on the quadrant of goal point in local frame (while alpha does not) 
    double sweepingAngle = 2 * alpha; 

    if (localGoal_.x < 0){
        if (localGoal_.y > 0) sweepingAngle = M_PI * 2 - sweepingAngle;
        if (localGoal_.y < 0) sweepingAngle = -(M_PI * 2 + sweepingAngle);
    }

    double turningRadius = linearDistanceToGoal / (2 * sin(std::abs(alpha)));

    // Update distancce and time 
    distance = std::abs(sweepingAngle) * turningRadius;
    time  = distance / CONSTANT_SPEED;

    // Update estimated goal pose
    estimatedGoalPose.position.x = goal.x;
    estimatedGoalPose.position.y = goal.y;
    estimatedGoalPose.yaw = Controller::normalizeAngle(origin.yaw + sweepingAngle);
 
    // Update steering 
    double steeringAngle = atan2(WHEELBASE * 2 * sin(alpha),linearDistanceToGoal);
    steering_ = steeringAngle * STEERING_RATIO;

    return std::abs(steeringAngle) < MAX_STEER_ANGLE;    
    
}

double Ackerman::distanceTravelled(void){
    return distanceTravelled_;
}

pfms::nav_msgs::Odometry Ackerman::getOdometry(void){
    pfms::geometry_msgs::Point lastPosition = {odo_.position.x, odo_.position.y};

    pipesPtr->read(odo_, platformType_);

    instantLinearDistanceToGoal_ = euclideanDistance(odo_.position.x, odo_.position.y, globalGoal_.x, globalGoal_.y);

    distanceTravelled_ += euclideanDistance(lastPosition.x, lastPosition.y, odo_.position.x, odo_.position.y);

    return odo_;
}

pfms::geometry_msgs::Point Ackerman::globalToLocalFrame(pfms::geometry_msgs::Point goal, pfms::geometry_msgs::Point robotPosition, double robotYaw) {
    // Calculate the relative position of the goal in the global frame
    double dx = goal.x - robotPosition.x;
    double dy = goal.y - robotPosition.y;

    // Convert the relative position to the robot frame using the robot's yaw angle
    double goal_x_robotFrame = dx * cos(robotYaw) + dy * sin(robotYaw);
    double goal_y_robotFrame = -dx * sin(robotYaw) + dy * cos(robotYaw);

    pfms::geometry_msgs::Point goal_robotFrame = {goal_x_robotFrame, goal_y_robotFrame};
    return goal_robotFrame;
}