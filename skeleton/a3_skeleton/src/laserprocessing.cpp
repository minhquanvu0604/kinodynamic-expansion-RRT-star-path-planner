
#include <algorithm>
#include <numeric>
#include <limits>

#include "laserprocessing.h"


using namespace std;

LaserProcessing::LaserProcessing()
{}

LaserProcessing::LaserProcessing(sensor_msgs::LaserScan laserScan):
    laserScan_(laserScan)
{}

void LaserProcessing::setLaserScan(sensor_msgs::LaserScan laserScan){
    laserScan_ = laserScan;
}

std::vector<int> LaserProcessing::conePointIndex(std::atomic<bool>* running){

    std::vector<int> segmentsIndex;
    bool previousValid = false;

    // Indices that belong to a single segment, cleared when having had completed one segment
    std::vector<int> oneCone;

    for (int i = 0; i < laserScan_.ranges.size(); i++){
        bool invalid = laserScan_.ranges.at(i) > laserScan_.range_max || laserScan_.ranges.at(i) < laserScan_.range_min;
        if (invalid){
            // Reached the end of a segment
            if (previousValid){
                
                // Filter the truck, recognize if the segment has fewer than 15 laser readings
                if (oneCone.size() < 30){
                    segmentsIndex.push_back(oneCone.at(oneCone.size() / 2));
                }

                // Truck ahead
                else {
                    int truckInd = oneCone.at(oneCone.size() / 2);
                    double dist = laserScan_.ranges.at(truckInd);
                    
                    // Stop if the truck is ahead
                    if (truckInd > 250 && truckInd < 390 && dist < 20){
                        // *running = false;
                        ROS_ERROR("STOPPED");
                    }
                }
                oneCone.clear();
                previousValid = false;
            }
            // Keep on finding nothing
            else {
                continue;
            }
        }
        // Valid readings - not inf, less then min, nan
        else{
            // Valid after valid, needs to examine more
            if (previousValid){
                double diff = laserScan_.ranges.at(i) - laserScan_.ranges.at(i-1);

                // Moving from far to near segment, for simplicity we just ignore the far segment
                if (diff > 0.3){
                    oneCone.clear();
                    previousValid = false;
                    continue;
                }          
                // Moving from near to far segment (end near segment), the near one is assumed to be a valid cone  
                else if (diff < -0.3){
                    if (oneCone.size() < 15){
                        oneCone.push_back(i);
                        segmentsIndex.push_back(oneCone.at(oneCone.size() / 2));
                        oneCone.clear();
                        previousValid = false;
                    }
                }
                // Continue on a segment
                else{
                    oneCone.push_back(i);
                    previousValid = true;
                }
            }
            // Start new segment
            else{
                oneCone.push_back(i);
                previousValid = true;
            }
        }
    }

    // Points filter for RRT, exclude out the cones from the side
    for (int i = 0; i < segmentsIndex.size(); i++){
        double range = laserScan_.ranges.at(segmentsIndex.at(i));
        if ((segmentsIndex.at(i) < 80 || segmentsIndex.at(i) > 560) && range > 20)
            segmentsIndex.erase(segmentsIndex.begin() + i);
    }
    return segmentsIndex;
}


std::vector<Point> LaserProcessing::getPoints(std::vector<int> conePointIndices, geometry_msgs::Pose pose){
    std::vector<Point> points;
    for (auto i : conePointIndices){
        auto geoPoint = polarToCart(i);
        Point pt = convertToPoint(geoPoint);
        Point globalPoint = localToGlobal(pt, pose);
        points.push_back(globalPoint);
    }
    return points;
}


double LaserProcessing::cross(const Point &O, const Point &A, const Point &B) {
    return (A.x - O.x) * (B.y - O.y) - (A.y - O.y) * (B.x - O.x);
}

std::vector<Point> LaserProcessing::convexHull(std::vector<Point> points) {
    int n = points.size(), k = 0;
    if (n <= 3) return points;
    std::vector<Point> hull(2*n);

    // Sort points lexicographically
    sort(points.begin(), points.end());

    // Build lower hull
    for (int i = 0; i < n; ++i) {
        while (k >= 2 && cross(hull.at(k-2), hull.at(k-1), points.at(i)) <= 0) k--;
        hull.at(k++) = points.at(i);
    }

    // Build upper hull
    for (int i = n-1, t = k+1; i > 0; --i) {
        while (k >= t && cross(hull.at(k-2), hull.at(k-1), points.at(i-1)) <= 0) k--;
        hull.at(k++) = points.at(i-1);
    }

    hull.resize(k-1);
    return hull;
}


bool LaserProcessing::pointInConvexPolygon(const std::vector<Point>& hull, Point p) {
    double windingNumber = 0; // Winding number
    for (int i = 0; i < hull.size(); i++) {
        Point p1 = hull.at(i);
        Point p2 = hull.at((i+1)%hull.size());

        // is p between p1 and p2 vertically?
        if (p1.y <= p.y) {
            if (p2.y > p.y) // an upward crossing
                if (cross(p1, p2, p) > 0) // p left of edge
                    windingNumber++; // have a valid up intersect
        } else {
            if (p2.y <= p.y) // a downward crossing
                if (cross(p1, p2, p) < 0) // p right of edge
                    windingNumber--; // have a valid down intersect
        }
    }
    return windingNumber != 0; // if winding number != 0, point is inside
}


std::vector<Point> LaserProcessing::getClosestPoints(const std::vector<Point>& points, const Point& target) {
    
    if(points.size() <= 6) {
        return points;
    }

    // Copy all points into another vector along with their distances to the target
    std::vector<std::pair<Point, double>> pointDistances;
    for(const auto& point : points) {
        double dist = std::sqrt(std::pow(target.x - point.x, 2) + std::pow(target.y - point.y, 2));
        pointDistances.push_back({point, dist});
    }

    // Sort the vector based on distances
    std::sort(pointDistances.begin(), pointDistances.end(),
              [](const std::pair<Point, double>& a, const std::pair<Point, double>& b) {
                  return a.second < b.second;
              });

    // Extract the first four points from the sorted vector
    std::vector<Point> closestPoints;
    for(int i = 0; i < 4; ++i) {
        closestPoints.push_back(pointDistances.at(i).first);
    }

    return closestPoints;
}



// ------------------------ HELPER FUNCTION ------------------------//

Point LaserProcessing::localToGlobal(Point localPoint, geometry_msgs::Pose pose){
    Point point;
    double yaw = normaliseAngle(tf::getYaw(pose.orientation));

    point.x = (localPoint.x * cos(yaw) - localPoint.y * sin(yaw) + pose.position.x + 3.7 * cos(yaw));
    point.y = (localPoint.x * sin(yaw) + localPoint.y * cos(yaw) + pose.position.y + 3.7 * sin(yaw));
    
    return point;
  }

double LaserProcessing::euclideanDistance(Point pt1, Point pt2){
    return std::pow(std::pow(pt1.x - pt2.x, 2) + std::pow(pt1.y - pt2.y, 2), 0.5);
}

double LaserProcessing::euclideanDistance(double x1, double y1, double x2, double y2){
    return std::pow(std::pow(x1 - x2, 2) + std::pow(y1 - y2, 2), 0.5);
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

double LaserProcessing::normaliseAngle(double theta) {
    if (theta > (2 * M_PI))
    theta = theta - (2 * M_PI);
    else if (theta < 0)
    theta = theta + (2 * M_PI);

    if (theta > M_PI){
        theta = -( (2* M_PI) - theta);
    }

    return theta;
}