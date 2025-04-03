#include "laserprocessing.h"
#include <algorithm>
#include <numeric>
#include <limits>
#include <map>

using namespace std;

LaserProcessing::LaserProcessing(sensor_msgs::LaserScan laserScan):
    laserScan_(laserScan)
{
}



//! @todo
//! TASK 1 - Refer to README.md and the Header file for full description
unsigned int LaserProcessing::countObjectReadings()
{
    unsigned int count=0;

    for (unsigned int i = 0; i < laserScan_.ranges.size(); i++){
        if (laserScan_.ranges.at(i) < laserScan_.range_max && laserScan_.ranges.at(i) > laserScan_.range_min)
            count++;
    }

    return count;
}


//! @todo
//! TASK 2 - Refer to README.md and the Header file for full description
unsigned int LaserProcessing::countSegments()
{
    unsigned int count=0;
    geometry_msgs::Point previousPoint;

    bool detected = false;

    for (unsigned int i = 1; i < laserScan_.ranges.size(); i++){

        // Invalid reading
        if (laserScan_.ranges.at(i) > laserScan_.range_max || laserScan_.ranges.at(i) < laserScan_.range_min){
            
            // Not detected anything
            if (detected == false) continue;

            // Complete 1 segment
            else {
                count++;
                detected = false;
            }
        }

        // Valid reading
        if (laserScan_.ranges.at(i) < laserScan_.range_max && laserScan_.ranges.at(i) > laserScan_.range_min){

            // Start new segment
            if (detected == false){
                detected = true;
                previousPoint = polarToCart(i);
                continue;
            }

            // Continue current segment
            else {
                geometry_msgs::Point pt = polarToCart(i);
                double dist = std::pow(std::pow(pt.x - previousPoint.x, 2) + std::pow(pt.y - previousPoint.y, 2), 0.5);
                previousPoint = pt;

                // Invalid points
                if (dist > 0.3) detected == false;

                continue;
            }
        }
    }



    //Check number
    ROS_INFO_STREAM("SEGMENT COUNT: " << count);

    return count;
}



//! @todo
//! TASK 3 - Refer to README.md and the Header file for full description
geometry_msgs::Point LaserProcessing::detectClosestCone(){

    geometry_msgs::Point previousPoint;

    // Save the current best
    unsigned int bestID = 0;
    double bestDist = std::numeric_limits<double>::max();

    bool detected = false;

    for (unsigned int i = 1; i < laserScan_.ranges.size(); i++){

        // Invalid reading
        if (laserScan_.ranges.at(i) > laserScan_.range_max || laserScan_.ranges.at(i) < laserScan_.range_min){
            
            // Not detected anything
            if (detected == false) continue;

            // Complete 1 segment
            else {
                // count++;
                detected = false;
            }
        }

        // Valid reading
        if (laserScan_.ranges.at(i) < laserScan_.range_max && laserScan_.ranges.at(i) > laserScan_.range_min){

            double dist = laserScan_.ranges.at(i);
            if (dist < bestDist){
                bestDist = dist;
                bestID = i;
            }

            // Start new segment
            if (detected == false){
                detected = true;
                previousPoint = polarToCart(i);
                continue;
            }

            // Continue current segment
            else {
                geometry_msgs::Point pt = polarToCart(i);
                double dist = std::pow(std::pow(pt.x - previousPoint.x, 2) + std::pow(pt.y - previousPoint.y, 2), 0.5);
                previousPoint = pt;

                // Invalid points
                if (dist > 0.3) detected == false;

                continue;
            }
        }

        

    }

    return polarToCart(bestID);
}


//! @todo
//! TASK 4 - Refer to README.md and the Header file for full description
geometry_msgs::Point LaserProcessing::detectRoadCentre(){

    geometry_msgs::Point previousPoint;

    // Save the current best
    unsigned int bestID = 0;
    unsigned int secondBestID = 0;

    double bestDist = std::numeric_limits<double>::max();
    double secondBestDist = std::numeric_limits<double>::max();


    bool detected = false;

    for (unsigned int i = 1; i < laserScan_.ranges.size(); i++){

        // Invalid reading
        if (laserScan_.ranges.at(i) > laserScan_.range_max || laserScan_.ranges.at(i) < laserScan_.range_min){
            
            // Not detected anything
            if (detected == false) continue;

            // Complete 1 segment
            else {
                // count++;
                detected = false;
            }
        }

        // Valid reading
        if (laserScan_.ranges.at(i) < laserScan_.range_max && laserScan_.ranges.at(i) > laserScan_.range_min){

            double dist = laserScan_.ranges.at(i);
            if (dist < bestDist){
                bestDist = dist;
                bestID = i;
            }
            else if (dist < secondBestDist && (dist - bestDist) > 0.3){
                secondBestDist = dist;
                secondBestID = i;
            }

            // Start new segment
            if (detected == false){
                detected = true;
                previousPoint = polarToCart(i);
                continue;
            }

            // Continue current segment
            else {
                geometry_msgs::Point pt = polarToCart(i);
                double dist = std::pow(std::pow(pt.x - previousPoint.x, 2) + std::pow(pt.y - previousPoint.y, 2), 0.5);
                previousPoint = pt;

                // Invalid points
                if (dist > 0.3) detected == false;

                continue;
            }
        }
    }

    auto firstCone = polarToCart(bestID);
    auto secondCone = polarToCart(secondBestID);

    geometry_msgs::Point midPoint;
    midPoint.x = (firstCone.x + secondCone.x) / 2;
    midPoint.y = (firstCone.y + secondCone.y) / 2;
    return midPoint;
}

void LaserProcessing::newScan(sensor_msgs::LaserScan laserScan){
    laserScan_=laserScan;
}


geometry_msgs::Point LaserProcessing::polarToCart(unsigned int index)
{
    float angle = laserScan_.angle_min + laserScan_.angle_increment*index;// + angle_range/2;
    float range = laserScan_.ranges.at(index);
    geometry_msgs::Point cart;
    cart.x = static_cast<double>(range*cos(angle));
    cart.y = static_cast<double>(range*sin(angle));
    return cart;
}

double LaserProcessing::angleConnectingPoints(geometry_msgs::Point p1, geometry_msgs::Point p2)
{
    return atan2(p2.y - p1.y, p2.x - p1.x);
}
