#ifndef DISPLAY_H
#define DISPLAY_H

#include <opencv2/opencv.hpp>

#include <iostream>
#include <string>
#include <vector>
#include <random>   // Includes the random number generator
#include "types.h"

using geometry_msgs::Pose;
using geometry_msgs::Point;
using geometry_msgs::RangeBearingStamped ;
using std::vector;

class Display
{
public:
    static const double AIRSPACE_SIZE;            /*!< The airspace size is (AIRSPACE_SIZE x AIRSPACE_SIZE) in metres */
    static const Point BSTATION_LOC;              /*!< The location of the base station */
    static const double OMEGA_MAX;              /*!< The maximum g-force */
    static const double V_MAX;                    /*!< The maximum velocity (m/s) */

    Display() : Display(4) {}                    /*!< Uses delegating constructor to invoke other implemented constructor */

    /*! @brief Constructor for class, positions of targets are drawn randomly
     *
     *  @param number of targets to simulate in the airspace
     */
    Display(unsigned int targets);

    /*! @brief Performs a scan from the pose
     *
     *  @param pose the pose to perform a scan from
     *  @return RangeBering readings reported by onboard radar from the pose supplied
     *  @note The readings are reported from closest range to furthest range
     */
    vector<RangeBearingStamped> scan(Pose pose);

private:
    /*! @brief Converts a Global coordinate to a pixel coordinate
     *
     *  @param ord The coordinate to convert.
     *  @return Point - The converted point.
     *  @note The coordinate x & y will round to the nearest 1 decimal place.
     */
    cv::Point convertToPoint(Point ord);

    /*! @brief Will draw point onto the airspace.
     *
     *  @param p The point to draw.
     *  @param colour The colour of the point.
     */
    void drawPoint(Point p, cv::Scalar colour);

    /*! @brief Will draw pose onto the airspace.
     *
     *  @param p The pose to draw.
     *  @param colour The colour of the point.
     */
    void drawPose(Pose p, cv::Scalar colour);

    /*! @brief Ensure that an angle is between (0 - 2pi).
     *
     *  @param theta The angle to normalise (radians)
     *  @return double - The normalised angle (radians).
     */
    double normaliseAngle(double theta);

    /*! @brief Calculates the euclidean distance between two coordiantes (on a plane ignoring z).
     *
     *  @param o1 The first coordinate.
     *  @param o2 The second coordinate.
     *  @return double - The distance between the two coordiantes.
     */
    double distance(Point o1, Point o2);

    cv::Mat airspace_;
    cv::Mat dst_;                               /*!< Mat object for output image file    */
    unsigned int pixel_map_size_;               /*!< Pixel size of the airspace */
    vector<Point> targets_;                     /*!< Targets in airspace */
    std::default_random_engine* generator_;     /*!< Pointer to the random number generator */

};

#endif // DISPLAY_H
