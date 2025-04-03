// add case where going in a straight line - cannot compute the turning centre
#ifndef ACKERMAN_HELPER_H
#define ACKERMAN_HELPER_H

#include <cmath>
#include <tuple>
#include "ackerman.h"


// Function prototypes
std::pair<double, double> turningCentre(double ini_x, double ini_y, double goal_x, double goal_y, double yaw);
std::tuple<double, double, double> headingLine (double yaw, double x, double y);

double euclideanDistance(double x1, double y1, double x2, double y2);
std::tuple<double, double, double> headingLine (double x, double y, double yaw);
std::tuple<double, double, double> lineFromPoints(double x1, double y1, double x2, double y2);
std::tuple<double, double, double> perpendicularBisector(double x1, double y1, double x2, double y2);
std::tuple<double, double, double> lineFromPointAndPerpendicularLine(double x, double y, std::tuple<double, double, double> perp_line);
std::pair<double, double> intersectionPoint(std::tuple<double, double, double> line1, std::tuple<double, double, double> line2);

double normalizeAngle(double angle);
double turnDirection(double current_x, double current_y, double current_yaw, double goal_x, double goal_y, double turningAngle);
                    


     ////////////////////////
    /// HELPER FUCNTIONS ///
   ////////////////////////

// Find the turning centre
std::pair<double, double> Ackerman::turningCentre(double ini_x, double ini_y, double goal_x, double goal_y, double yaw){

    std::tuple<double, double, double> bisector = Ackerman::perpendicularBisector(ini_x, ini_y, goal_x, goal_y);

    std::tuple<double, double, double> heading = Ackerman::headingLine(ini_x, ini_y, yaw);

    std::tuple<double, double, double> line = Ackerman::lineFromPointAndPerpendicularLine(ini_x, ini_y, heading);

    return intersectionPoint(bisector, line);
    }

// Find the heading line equation from yaw angle and position of the car
std::tuple<double, double, double> Ackerman::headingLine (double x, double y, double yaw) {
    double slope = tan(yaw);
    double y_intercept = y - slope * x;
    
    // Convert to homogeneous form: Ax + By + C = 0
    double A = -slope;
    double B = 1;
    double C = -y_intercept;

    return std::make_tuple(A, B, C);
}



     ///////////////////////
    /// SCALAR GEOMETRY ///
   ///////////////////////

// Compute the distance from 2 poinst
double Ackerman::euclideanDistance(double x1, double y1, double x2, double y2) {
    return std::sqrt(std::pow(x2 - x1, 2) + std::pow(y2 - y1, 2));
}

// Compute line equation from the coordinates of 2 points
std::tuple<double, double, double> Ackerman::lineFromPoints(double x1, double y1, double x2, double y2) {
    double A = y1 - y2;
    double B = x2 - x1;
    double C = x1 * y2 - x2 * y1;

    return std::make_tuple(A, B, C);
}

// Compute the equation of perpendicular bisector from the coordinates of 2 points
std::tuple<double, double, double> Ackerman::perpendicularBisector(double x1, double y1, double x2, double y2) {
    double mx = (x1 + x2) / 2.0;
    double my = (y1 + y2) / 2.0;
    double A = x2 - x1;
    double B = y2 - y1;
    double C = -(A * mx + B * my);

    return std::make_tuple(A, B, C);
}

// Compute the equation of a line given a point on it and a line that it is perpendicular to
std::tuple<double, double, double> Ackerman::lineFromPointAndPerpendicularLine(double x, double y, std::tuple<double, double, double> perp_line) {
    double A, B, C;
    std::tie(A, B, C) = perp_line;
    double new_A = B;
    double new_B = -A;
    double new_C = -(new_A * x + new_B * y);

    return std::make_tuple(new_A, new_B, new_C);
}

// Find the intersection point of 2 given lines
std::pair<double, double> Ackerman::intersectionPoint(std::tuple<double, double, double> line1, std::tuple<double, double, double> line2) {
    double A1, B1, C1, A2, B2, C2;
    std::tie(A1, B1, C1) = line1;
    std::tie(A2, B2, C2) = line2;

    double det = A1 * B2 - A2 * B1;

    if (det == 0) {
        // The lines are parallel or coincident
        throw std::runtime_error("The lines are parallel or coincident");
    }

    double x = (B1 * C2 - B2 * C1) / det;
    double y = (A2 * C1 - A1 * C2) / det;

    return std::make_pair(x, y);
}


     ///////////////////////
    /// VECTOR GEOMETRY ///
   ///////////////////////

// Normalize angle, making it range from -M_PI to M_PI
double Ackerman::normalizeAngle(double angle) {
    angle = fmod(angle, 2 * M_PI);
    if (angle > M_PI) {
        angle -= 2 * M_PI;
    } else if (angle <= -M_PI) {
        angle += 2 * M_PI;
    }
    return angle;
}

// Determine whether the robot needs to turn left or right and add the angle
double Ackerman::turnDirection(double current_x, double current_y, double current_yaw,
                    double goal_x, double goal_y, double turningAngle) {
    // Calculate the vector from the robot's position to the target position
    double dx = goal_x - current_x;
    double dy = goal_y - current_y;

    // Calculate the orientation of the target vector
    double target_yaw = atan2(dy, dx);

    // Normalize the orientations to the range [-pi, pi]
    current_yaw = normalizeAngle(current_yaw);
    target_yaw = normalizeAngle(target_yaw);

    double yaw_difference = normalizeAngle(target_yaw - current_yaw);

    // Determine the turn direction
    if (yaw_difference > 0) 
        return normalizeAngle(current_yaw + turningAngle);
    else if (yaw_difference < 0) 
        return normalizeAngle(current_yaw - turningAngle);
    else if (yaw_difference = 0)
        return current_yaw;
}
                    
#endif


