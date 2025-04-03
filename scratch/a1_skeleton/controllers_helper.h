#ifndef CONTROLLERS_HELPER_H
#define CONTROLLERS_HELPER_H

#include <iostream>
#include <eigen3/Eigen/Dense>
#include <cmath>
#include <tuple>    

// FUNCTION PROTOTYPES
// Eigen::Matrix3d Ackerman::create2DTransformationMatrix(const Eigen::Vector2d& translation, double rotation_angle_rad);
// Eigen::Vector2d Ackerman::convertPointFromGlobalToLocal(double robot_x, double robot_y, double point_x, double point_y, double localFrameRotationAngleRad);


     ////////////////////////
    /// HELPER FUCNTIONS ///
   ////////////////////////

// Find turning centre
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

Eigen::Matrix3d Ackerman::create2DTransformationMatrix(const Eigen::Vector2d& translation, double rotation_angle_rad) {
    Eigen::Matrix3d transformation = Eigen::Matrix3d::Identity();
    transformation(0, 0) = cos(rotation_angle_rad);
    transformation(0, 1) = -sin(rotation_angle_rad);
    transformation(1, 0) = sin(rotation_angle_rad);
    transformation(1, 1) = cos(rotation_angle_rad);
    transformation.block<2, 1>(0, 2) = translation;
    return transformation;
}

Eigen::Vector2d Ackerman::convertPointFromGlobalToLocal(double robot_x, double robot_y, double point_x, double point_y, double localFrameRotationAngleRad) {
    
    // Define the point in the global frame
    Eigen::Vector2d pointGlobal(point_x, point_y);
    
    // Define the translation vector and rotation angle for the local frame with respect to the global frame
    Eigen::Vector2d localFrameTranslation(robot_x, robot_y);
    
    // Create the 2D homogeneous transformation matrix for the local frame with respect to the global frame
    Eigen::Matrix3d transformationLocalToGlobal = create2DTransformationMatrix(localFrameTranslation, localFrameRotationAngleRad);

    // Convert the point from the global frame to the local frame
    Eigen::Vector3d pointGlobalHomogeneous(pointGlobal(0), pointGlobal(1), 1.0); // Homogeneous coordinates (x, y, 1)
    Eigen::Vector3d pointLocalHomogeneous = transformationLocalToGlobal.inverse() * pointGlobalHomogeneous;

    // Extract and return the first two elements of the resulting vector (x, y coordinates)
    return pointLocalHomogeneous.head<2>();
}

// Convert the point from the global frame to the local frame
// Eigen::Vector2d point_local = convertPointFromGlobalToLocal(1, 1, -3, 3, -M_PI/2);


#endif