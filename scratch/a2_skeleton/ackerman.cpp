#include <cmath>
#include <iostream>

#include "ackerman.h"
#include "pipes.h"


Ackerman::Ackerman() {
    platformType_ = pfms::PlatformType::ACKERMAN;
    // Initialize some variables 
    // platformType_ = pfms::PlatformType::ACKERMAN;
    // distanceTravelled_ = 0;
    // timeInMotion_ = 0;
}
 

// pfms::nav_msgs::Odometry Ackerman::getOdometry(void){
//     // pfms::geometry_msgs::Point lastPosition = {odo_.position.x, odo_.position.y};

//     // pipesPtr->read(odo_, platformType_);

//     // instantLinearDistanceToGoal_ = euclideanDistance(odo_.position.x, odo_.position.y, globalGoal_.x, globalGoal_.y);

//     // distanceTravelled_ += euclideanDistance(lastPosition.x, lastPosition.y, odo_.position.x, odo_.position.y);

//     return odo_;
// }

