#include "controller.h"
#include "pipes.h"

Controller::Controller() : pipesPtr_{new Pipes()}, tolerance_{0.5}, 
                        platformStatus_{pfms::PlatformStatus::IDLE}{
    
    // Create the pipe 
    // pipesPtr = new Pipes();
}

pfms::PlatformStatus Controller::status(void){
    return platformStatus_; 
}

pfms::PlatformType Controller::getPlatformType(void){
    return platformType_;
}

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

pfms::nav_msgs::Odometry Controller::getOdometry(void){
    // pfms::geometry_msgs::Point lastPosition = {odo_.position.x, odo_.position.y};

    pipesPtr_->read(odo_, platformType_);

    // instantLinearDistanceToGoal_ = euclideanDistance(odo_.position.x, odo_.position.y, globalGoal_.x, globalGoal_.y);

    // distanceTravelled_ += euclideanDistance(lastPosition.x, lastPosition.y, odo_.position.x, odo_.position.y);

    return odo_;
}


// --------------// HELPER FUNCTION // --------------------------------------------------------//

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

double Controller::euclideanDistance(double x1, double y1, double x2, double y2) {
    return std::sqrt(std::pow(x2 - x1, 2) + std::pow(y2 - y1, 2));
}