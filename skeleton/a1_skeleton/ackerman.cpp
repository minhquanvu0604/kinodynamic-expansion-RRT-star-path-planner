#include <cmath>
#include <limits>
#include "ackerman.h"
#include "controller_helper.h"
#include "pipes.h"

const double fixThrottle = 0.2;

Ackerman::Ackerman(){
    type_ = pfms::PlatformType::ACKERMAN;
}

//1
bool Ackerman::reachGoal(void){

    bool OK  =  pipesPtr->read(odo_,type_);

    const double steeringAngle = atan(WHEELBASE / turningRadius_);

    const double lockToLockRev = steeringAngle * STEERING_RATIO / M_PI;

    // Start sending commands
    unsigned long i = 0;

    while(true) {
        pfms::commands::UGV ugv {
                    i,
                    0,
                    lockToLockRev,
                    fixThrottle
                 };
        pipesPtr->send(ugv);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        

        // // // // // // Keep these for debugging // // // //
        // std::cout << "wrote:" << i << std::endl;
        // bool OK  =  pipesPtr->read(odo_,type_);
        // if(OK){
        //     std::cout << "i seq x,y,yaw,vx,vy: " <<
        //         i << " " <<
        //         odo_.seq << " " <<
        //         odo_.position.x << " " <<
        //         odo_.position.y << " " <<
        //         odo_.yaw << " " <<
        //         odo_.linear.x << " " <<
        //         odo_.linear.y << std::endl;
        // }
        
        // std::this_thread::sleep_for (std::chrono::milliseconds(10));
        // // // // // // // // // // // // // // // // // // //
        
        i++;
        
        if (euclideanDistance(odo_.position.x, odo_.position.y, goal_.x, goal_.y) < 2) break;
    } 
        
    std::this_thread::sleep_for (std::chrono::milliseconds(10));

    // Starting slowing down
    while(true){
        double distance = euclideanDistance(odo_.position.x, odo_.position.y, goal_.x, goal_.y);

        pfms::commands::UGV ugv {
                    i,
                    4000 * (2 - distance), // P controller -> UPGRADE?
                    lockToLockRev,
                    0
                 };
        pipesPtr->send(ugv);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        
        std::cout << "Slowing down..." << std::endl;

        if (odo_.linear.x < std::numeric_limits<double>::epsilon() 
            && odo_.linear.y < std::numeric_limits<double>::epsilon()) break;
    }

    return true;
}

// 2 DONE
bool Ackerman::setGoal(pfms::geometry_msgs::Point commandGoal){
    // Update internal variables
    goal_.x = commandGoal.x; 
    goal_.y  = commandGoal.y;

    // Fetch odometry
    pipesPtr->read(odo_,type_);

    // Find the turning radius
    std::pair<double, double> carTurningCentre = turningCentre(odo_.position.x, odo_.position.y, goal_.x, goal_.y, odo_.yaw);
    
    turningRadius_ =  euclideanDistance(odo_.position.x, odo_.position.y, carTurningCentre.first, carTurningCentre.second);
    
    return turningRadius_ > r_min;
}

// 3 
bool Ackerman::checkOriginToDestination(pfms::nav_msgs::Odometry origin, // IF CURRENT POINT THEN DON'T HAVE TO SET PARA
                                        pfms::geometry_msgs::Point goal,
                                        double& distance,
                                        double& time,
                                        pfms::nav_msgs::Odometry& estimatedGoalPose){
    
    bool OK = Ackerman::setGoal(goal);                                        
    
    std::pair<double, double> carTurningCentre = turningCentre(origin.position.x, origin.position.y,
                                                                goal.x, goal.y, origin.yaw);
    double turningRadius =  euclideanDistance(origin.position.x, origin.position.y,
                                                        carTurningCentre.first, carTurningCentre.second);
    double linearDistanceToGoal = euclideanDistance(origin.position.x, origin.position.y, goal.x, goal.y);
                                                            
    double turningAngle = 2 * asin(linearDistanceToGoal / (2 * turningRadius));

    distance = turningAngle * turningRadius;

    time = distance / CONSTANT_SPEED;

    estimatedGoalPose.position.x = goal.x;
    estimatedGoalPose.position.y = goal.y;
    estimatedGoalPose.yaw = turnDirection(odo_.position.x, odo_.position.y, odo_.yaw,
                    goal.x, goal.y, turningAngle);
    
    return turningRadius > r_min;                                               
                                        }

// 5
double Ackerman::distanceToGoal(void){
    checkOriginToDestination(odo_, goal_, distance_, time_, estimatedGoalPose_);
    return distance_;                                         
}
