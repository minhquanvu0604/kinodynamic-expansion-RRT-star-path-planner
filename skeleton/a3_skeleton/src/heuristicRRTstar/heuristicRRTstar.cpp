#include "heuristicRRTstar/heuristicRRTstar.h"
#include <random>
#include <vector>
#include <cmath>
#include <algorithm>
#include <limits>
#include <iostream>
#include <yaml-cpp/yaml.h>


RRTStar::RRTStar(ros::NodeHandle& nh, long& id) : RRTStar(nh, id, Node(), std::vector<Point>(), std::vector<Point>()) {}

RRTStar::RRTStar(ros::NodeHandle& nh, long& id, Node start, const std::vector<Point>& obstacleList, const std::vector<Point>& rrtConeTargets) :
    nh_{nh}, id_{id}, ct_{0}, gen_{std::random_device{}()}
    // RRTStar(nh, id, start, 1000, 35, 20, 4, 1.5, true, 3, 7, 10, obstacleList, rrtConeTargets) 
{
    markerPub_ = nh_.advertise<visualization_msgs::MarkerArray>("visualization_marker",1000,false);
    
    // Setting RRT parameters
    std::string file_path = "/home/quanvu/git/pfms-2023a-minhquanvu0604/skeleton/a3_skeleton/config/RRTparams.yaml";
    YAML::Node params = YAML::LoadFile(file_path);
    
    iteration_ = params["iteration"].as<double>(); 
    planDistance_ = params["planDistance"].as<double>();  

    turnAngle_ = params["turnAngle"].as<double>(); 
    turnAngle_ = turnAngle_ * M_PI / 180;

    expandDist_ = params["expandDist"].as<double>(); 
    animation_ = params["animation"].as<bool>(); 
    obstacleSize_ = params["obstacleSize"].as<double>(); 
    maxRangeFromTarget_ = params["maxRangeFromTarget"].as<double>(); 
    neighboorhoodRange_ = params["neighboorhoodRange"].as<double>(); 
    distanceToExclude_ = params["distanceToExclude"].as<double>();

    // Show the current configuration of the RRT
    std::cout << "----RRT CONFIG----" << std::endl;
    std::cout << "iteration: " << params["iteration"].as<double>() << std::endl;
    std::cout << "planDistance: " << params["planDistance"].as<double>() << std::endl;
    std::cout << "turnAngle: " << turnAngle_ << std::endl;
    std::cout << "expandDist: " << params["expandDist"].as<double>() << std::endl;
    std::cout << "animation: " << params["animation"].as<bool>() << std::endl;
    std::cout << "obstacleSize: " << params["obstacleSize"].as<double>() << std::endl;
    std::cout << "maxRangeFromTarget: " << params["maxRangeFromTarget"].as<double>() << std::endl;
    std::cout << "neighboorhoodRange: " << params["neighboorhoodRange"].as<double>() << std::endl;
    std::cout << "distanceToExclude: " << params["distanceToExclude"].as<double>() << std::endl;
}

// RRTStar::RRTStar(ros::NodeHandle& nh, long& id, Node start, double iteration, double planDistance, double turnAngle, double neighboorhoodRange,
//                 double expandDist, bool animation, double obstacleSize, double maxRangeFromTarget, double distanceToExclude,
//                 const std::vector<Point>& obstacleList, const std::vector<Point>& rrtConeTargets) :

//     nh_{nh}, id_{id}, ct_{0}, start_{start}, iteration_{iteration}, planDistance_{planDistance}, turnAngle_{turnAngle}, neighboorhoodRange_{neighboorhoodRange},
//     expandDist_{expandDist}, animation_{animation}, obstacleSize_{obstacleSize}, maxRangeFromTarget_{maxRangeFromTarget},
//     obstacleList_{obstacleList}, rrtConeTargets_{rrtConeTargets}, gen_{std::random_device{}()}

// {
//     markerPub_ = nh_.advertise<visualization_msgs::MarkerArray>("visualization_marker",1000,false);
//     turnAngle_ = turnAngle_ * M_PI / 180;
// }

void RRTStar::setData(Node start, const std::vector<Point>& obstacleList, const std::vector<Point>& rrtConeTargets){
    start_ = start;
    obstacleList_ = obstacleList;
    rrtConeTargets_ = rrtConeTargets;
}

void RRTStar::createTree() {

    nodeList_.clear();
    leafNodes_.clear();

    nodeList_.push_back(start_);
    int count = 0;
    int collision = 0;
    int outsidePlan = 0;
    int alreadyCreated = 0;

    // For drawing all edges
    visualization_msgs::MarkerArray edgeArray;

    // Loop over all the iterations
    for (int i = 0; i < iteration_; i++) {
        
        setTargetCone();
        Point randomPoint = getRandomPointFromTargetList();

        int nearestIndex = getNearestListIndex(nodeList_, randomPoint);
        if (nearestIndex == -1) continue;
        Node nearestNode = nodeList_.at(nearestIndex);



        // This node is outside of the plan area
        if (nearestNode.cost >= planDistance_) {
            outsidePlan++;
            continue;
        }



        // Constraint the angle
        Node newNode = angularConstraint(randomPoint, nearestIndex);
        
        // // Skip the iteration if the node is already created
        // if (std::find(nodeList_.begin(), nodeList_.end(), newNode) != nodeList_.end()) {
        //     alreadyCreated++;
        //     continue;
        // }
        
        // Collision check
        bool noCollision = collisionCheck(newNode);
        if (!noCollision) {
            collision++;
            continue;
        }

        // Find indices of closest neighbors
        std::vector<int> nearInds = findNeighboors(newNode);

        // Choose parent for neighboor nodes
        chooseParent(newNode, nearInds);

        // if (newNode.parent == -1){
        //     collision++;
        //     continue;
        // }
            
        // Join the node list here
        nodeList_.push_back(newNode);
        // Its parent is not the last node
        // nodeList_.at(newNode.parent).lastNode = false;
        
        // Become leaf node
        double distToNode = euclideanDistance(start_.x, start_.y, newNode.x, newNode.y);
        if (newNode.cost >= planDistance_ || distToNode > 32) 
            leafNodes_.push_back(newNode);

        count++;

        // Rewire - RRT*
        int newNodeInd = nodeList_.size() - 1;
        rewire(newNode, newNodeInd, nearInds);    
    }
    
    // DEBUG
    visualization_msgs::MarkerArray nodeArray;
    for (auto node : nodeList_){
        auto nodeMarker = createNodeMarker(node);
        nodeArray.markers.push_back(nodeMarker);
    }
    markerPub_.publish(nodeArray);


    publishMarkers();
}

std::vector<Node> RRTStar::getNodeList(){
    return nodeList_;
}

std::vector<Node> RRTStar::getLeafNodes(){
    return leafNodes_;
}

// Point RRTStar::getRandomPoint() {

//     std::uniform_real_distribution<> dist1(0, planDistance_);
//     std::uniform_real_distribution<> dist2(-planDistance_, planDistance_);

//     double randX = dist1(gen_);
//     double randY = dist2(gen_);

//     std::vector<double> rnd = {randX, randY};

//     std::vector<std::vector<double>> rotationMatrix = {{std::cos(start_.yaw), -std::sin(start_.yaw)}, 
//                                                   {std::sin(start_.yaw), std::cos(start_.yaw)}};

//     Point rotatedRnd;
//     rotatedRnd.x = rotationMatrix.at(0).at(0) * rnd.at(0) + rotationMatrix.at(0).at(1) * rnd.at(1) + start_.x;
//     rotatedRnd.y = rotationMatrix.at(1).at(0) * rnd.at(0) + rotationMatrix.at(1).at(1) * rnd.at(1) + start_.y;

//     return rotatedRnd;
// }


Point RRTStar::getRandomPointFromTargetList() {

    // Choose a random point from the RRT cone targets 
    int targetId = std::uniform_int_distribution<>(0, rrtConeTargets_.size() - 1)(gen_);

    double randAngle = std::uniform_real_distribution<>(0.0, 2.0 * M_PI)(gen_);
    double randDist = std::uniform_real_distribution<>(obstacleSize_, maxRangeFromTarget_)(gen_);
        
    // Coordinate the point target 
    double x = rrtConeTargets_.at(targetId).x;
    double y = rrtConeTargets_.at(targetId).y;
    // Coordinate of the random point        
    return {x + randDist * cos(randAngle), y + randDist * sin(randAngle)};
}


Node RRTStar::angularConstraint(const Point& randomPoint, int nearestIndex) {

    Node nearestNode = nodeList_.at(nearestIndex);
    Node nearestNodeParent ;

    if (nearestNode.parent != -1)
        nearestNodeParent = nodeList_.at(nearestNode.parent);
    else {
        nearestNodeParent = start_;
        }
        
    double angleChange = calculateAngle(nearestNode.x, nearestNode.y, nearestNodeParent.x, nearestNodeParent.y,
                                        randomPoint.x, randomPoint.y, nearestNode.x, nearestNode.y);

    if (angleChange > turnAngle_)
        angleChange = turnAngle_;
    // else if (angleChange >= -turnAngle_)
    //     angleChange = 0;
    // else
    //     angleChange = -turnAngle_;
    else if (angleChange < -turnAngle_)
        angleChange = -turnAngle_;    

    Node newNode = nearestNode; 

    newNode.yaw += angleChange;
    newNode.x += expandDist_ * std::cos(newNode.yaw);
    newNode.y += expandDist_ * std::sin(newNode.yaw);

    newNode.cost += expandDist_;
    newNode.parent = nearestIndex;

    return newNode;
}

double RRTStar::normalizeAngle(double angle) {
    return std::fmod(angle + M_PI, 2.0 * M_PI) - M_PI;
}

double RRTStar::euclideanDistance(double x1, double y1, double x2, double y2){
    return std::pow(std::pow(x1 - x2,2) + std::pow(y1 - y2,2),0.5);
}

int RRTStar::getNearestListIndex(const std::vector<Node>& nodeList, const Point& randomPoint){
    double minDist = std::numeric_limits<double>::max();
    int minIndex = -1;
    
    for (int i = 0; i < nodeList.size(); i++){
        double dist = euclideanDistance(randomPoint.x, randomPoint.y, nodeList.at(i).x, nodeList.at(i).y);
        
        if (dist < minDist){
            minDist = dist;
            minIndex = i;
        }
    }
    return minIndex;
}

bool RRTStar::collisionCheck(const Node& node){
    for (const auto& ob : obstacleList_){
        double dist = euclideanDistance(node.x, node.y, ob.x, ob.y);
        if (dist < obstacleSize_) return false;
    }
    return true;
}

std::vector<int> RRTStar::findNeighboors(const Node& newNode) 
{
    std::vector<int> neighboorIndices;

    for (int i = 0; i < nodeList_.size(); i++) 
    {
        double dist = euclideanDistance(newNode.x, newNode.y, nodeList_.at(i).x, nodeList_.at(i).y);

        if (dist <= neighboorhoodRange_) 
        {
            neighboorIndices.push_back(i);
        }
    }

    return neighboorIndices;
}

void RRTStar::chooseParent(Node& newNode, std::vector<int> nearIndices) {
    
    double minCost = std::numeric_limits<double>::max();
    int parentID = -1;

    // Loop over all the neighboors
    for (int i = 0; i < nearIndices.size(); i++) {

        // Simplification: if 2 node don't collide with any obstacle, assume the edge between them also doesn't collide 
        Node neighboor = nodeList_.at(nearIndices.at(i));
               
        bool connectionOK = isPathFree(newNode, neighboor);
        if (!connectionOK) 
            continue;

        double dist = euclideanDistance(newNode.x, newNode.y, neighboor.x, neighboor.y);
        double cost = dist + neighboor.cost;

        if (cost < minCost){
            minCost = cost;
            parentID = nearIndices.at(i);
        }
    }
    // Update
    newNode.parent = parentID;

    if (parentID == -1 ) return;

    newNode.cost = minCost;

    // Update total turning
    Node parent = nodeList_.at(newNode.parent);
    Node greatParent;

    if (parent.parent > 0){
        greatParent = nodeList_.at(parent.parent);
    }
    else greatParent = start_;

    double angleTurned = calculateAngle(newNode.x, newNode.y, parent.x, parent.y, parent.x, parent.y, greatParent.x, greatParent.y);
    // newNode.turning += std::abs(angleTurned);
}


// YOU CAN DO THE SORTING OF DISTANCE TO ACKERMAN HERE IF YOU WANT
void RRTStar::setTargetCone(){

    rrtConeTargets_.clear();

    for (auto& ob : obstacleList_){
        double dist = euclideanDistance(start_.x, start_.y, ob.x, ob.y);

        if (dist > distanceToExclude_)
            rrtConeTargets_.push_back(ob);
    }
}

// Only consider completed path - ones with leaf node
std::vector<Point> RRTStar::chooseBestPath()
{
    // std::vector<double> pathConfidence;
    std::vector<Point> bestPath;

    double bestRating = std::numeric_limits<double>::min();
    Node* bestLeaf = nullptr;

    // Tuning the reward system
    // TUNING
    const double vicinity = 7;
    // const double vicinitySq = std::pow(vicinity, 2);// Squared number of vicinity range of 8
    const double goThroughPairBonus = 1000.0;
    const double turningPenaltyFactor = 0.5;

    int bestPathInd = -1;
    std::vector<std::vector<int>> LEFT;
    std::vector<std::vector<int>> RIGHT;
    double bestDistReward = -1;
    double bestPairReward = -1;
    int pathInd = -1;

    for(auto& leaf : leafNodes_) {
        // -------------------------ONE WHOLE PATH---------------------//
        pathInd++;

        // SET UP FOR DEBUGGING
        std::vector<int> leftConeNumberNodes;
        std::vector<int> rightConeNumberNodes;
        
        // Starting node
        Node* node = &leaf;

        double pathConfidence = 0;
        double distRewardPath = 0;
        double conePairRewardPath = 0;

        int numNode = 0;

        // Loop through all the nodes
        while(node->parent != -1) { // assuming -1 indicates no parent

            // --------- ONE SINGLE NODE ------------//
            numNode++;
            double distRewardNode = 0;
            double conePairRewardNode = 0;

            // std::vector<Point> leftCones;
            // std::vector<Point> rightCones;
            int leftCones = 0;
            int rightCones = 0;

            int pointInVic = 0;

            // Cone target or ob list?
            for(auto& cone : rrtConeTargets_) {

                // --------- ONE CONE OF CURRENT NODE ------------//
                // Rewarding for nodes
                double distSq = std::pow(cone.x - node->x, 2) + std::pow(cone.y - node->y, 2);
                double dist = std::sqrt(distSq);
                
                if(dist < vicinity) {

                    pointInVic++;

                    // ------------ TUNING 1--------------//
                    distRewardNode += distSq;

                    // ------------TUNING 1.5 ------------//
                    
                    Point localCone = globalToLocalFrame(cone, start_);
                    if (std::abs(localCone.y) < 10)
                        distRewardNode *= 100;

                    // ------------ TUNING 2--------------//
                    if (node->parent != -1){
                        if(isConeLeftFromNode(*node, cone)) 
                            leftCones++;
                        else 
                            rightCones++;                      
                    }
                }       
            }
            if (pointInVic != 0) { //< Else move on to next node
                
                leftConeNumberNodes.push_back(leftCones);
                rightConeNumberNodes.push_back(rightCones);
            
                // Rewarding nodes that lies between two cones
                int numPairCone = std::min(leftCones, rightCones);
                conePairRewardNode = numPairCone * goThroughPairBonus;

                conePairRewardPath += conePairRewardNode;
                distRewardPath += distRewardNode;
            }
            
            // Make sure the parent index is valid before accessing it.
            if(node->parent >= 0 && node->parent < nodeList_.size()) {

                node = &nodeList_.at(node->parent);
            }
            else 
                break;
            
            // ===========ONE SINGLE NODE=============//
        }
        LEFT.push_back(leftConeNumberNodes);
        RIGHT.push_back(rightConeNumberNodes);

        pathConfidence = distRewardPath + conePairRewardPath;        

        if(pathConfidence > bestRating) {
            bestRating = pathConfidence;
            
            bestLeaf = &leaf;
            bestPathInd = pathInd;

            bestDistReward = distRewardPath;
            bestPairReward = conePairRewardPath;
        }
    }

    // NO BEST PATH
    if (pathInd == -1) return bestPath;

    // Trace back to get the best path    
    if(bestLeaf != nullptr) {
        Node* node = bestLeaf;

        while(node->parent > 0 && node->parent < nodeList_.size()) {

            Point point(node->x, node->y);

            bestPath.push_back(point);  

            // Make sure the parent index is valid before accessing it.
            if(node->parent >= 0 && node->parent < nodeList_.size()) {
                node = &nodeList_.at(node->parent);
            }
            else {
                break;
            }
        }
        std::reverse(bestPath.begin(), bestPath.end());  // To get the path from start to leaf.
    }

    if (bestPath.size() == 0) 
        ROS_ERROR("NO BEST PATH");

    if (!bestPath.empty()){
        // Markeres
        auto bestPathMarker = createBestPathMarker(bestPath);
        markerPub_.publish(bestPathMarker);
    }

    return bestPath;
}


bool RRTStar::isConeLeftFromNode(Node node, Point cone){
    if (node.parent >= 0){
        Node parentNode = nodeList_.at(node.parent);
        return (node.x - parentNode.x) * (cone.y - parentNode.y) - (node.y - parentNode.y) * (cone.x - parentNode.x) > 0;
    }
    else return false;
        
}



double RRTStar::calculateAngle(double x1, double y1, double x2, double y2, double x3, double y3, double x4, double y4) {
    // Form vectors from points
    double Ax = x2 - x1;
    double Ay = y2 - y1;
    double Bx = x4 - x3;
    double By = y4 - y3;

    // Calculate dot product
    double dotProduct = Ax * Bx + Ay * By;
    
    // Calculate the determinant (used to find the direction of the angle)
    double det = Ax * By - Ay * Bx;

    // Calculate the angle
    double angle = std::atan2(det, dotProduct);

    // Convert to degrees
    angle *= 180.0 / M_PI;

    return angle;
}

void RRTStar::rewire(Node& newNode, int newNodeInd, const std::vector<int>& nearInds) {
    
    for (int i = 0; i < nearInds.size(); ++i) {
        Node nearNode = nodeList_.at(nearInds.at(i));
        double altCost = newNode.cost + euclideanDistance(newNode.x, newNode.y, nearNode.x, nearNode.y);
        
        Node newNodeParent = nodeList_.at(newNode.parent);

        // If the node newNode provides shorter path
        if (altCost < nearNode.cost) {

            nearNode.parent = newNodeInd;
            nearNode.cost = altCost;
            // nearNode.turning += std::abs(calculateAngle(newNodeParent.x, newNodeParent.y, newNode.x, newNode.y, newNode.x, newNode.y, nearNode.x, nearNode.y));
            // nearNode.lastNode = true;
            // newNode.lastNode = false;

            nodeList_.at(nearInds.at(i)) = nearNode;
        }
    }
}

std::vector<Point> RRTStar::calculateMovingAveragePath(const std::deque<std::vector<Point>>& paths) {
    std::vector<Point> avgPath;
    
    if (paths.empty()) {
        return avgPath;
    }

    const int pathLength = 7;    
    for (int i = 0; i < pathLength; i++) {
        Point sumPoint;

        for (const auto& path : paths) {

            if (i >= path.size()) continue;

            sumPoint.x += path.at(i).x;
            sumPoint.y += path.at(i).y;
        }
        

        Point avgPoint;
        avgPoint.x = sumPoint.x / paths.size();
        avgPoint.y = sumPoint.y / paths.size();
        
        avgPath.push_back(avgPoint);
    }
    
    
    // Adding start point to the path because the avgPath function does not include it

    std::vector<Point> first = std::vector<Point>(avgPath.begin(), avgPath.begin() + 6);

    auto markerArray = createAvgPathMarker(first);
    markerPub_.publish(markerArray);

    auto marker = createHeadpointMarker(avgPath);
    markerPub_.publish(marker);


    return avgPath;

    return avgPath;
}

void RRTStar::updatePaths(std::deque<std::vector<Point>>& paths, const std::vector<Point>& newPath) {
    const int maxNumPaths = 10;
    
    paths.push_back(newPath);
    
    if (paths.size() > maxNumPaths) {
        paths.pop_front();
    }
}

Point RRTStar::globalToLocalFrame(Point goal, Node robot) {
    // Calculate the relative position of the goal in the global frame
    double dx = goal.x - robot.x;
    double dy = goal.y - robot.y;

    Point goalRobotFrame;
    
    // Convert the relative position to the robot frame using the robot's yaw angle
    goalRobotFrame.x = dx * cos(robot.yaw) + dy * sin(robot.yaw);
    goalRobotFrame.y = -dx * sin(robot.yaw) + dy * cos(robot.yaw);

    return goalRobotFrame;
}

bool RRTStar::isPathFree(const Node& newNode, const Node& nearestNode) {
    double resolution = expandDist_ / 10; // You can adjust this based on the desired precision

    double dx = newNode.x - nearestNode.x;
    double dy = newNode.y - nearestNode.y;

    double dist = std::sqrt(dx * dx + dy * dy);
    double stepCount = dist / resolution;

    for (int i = 0; i <= stepCount; ++i) {
        double ratio = i / stepCount;

        Node checkNode;
        checkNode.x = nearestNode.x + ratio * dx;
        checkNode.y = nearestNode.y + ratio * dy;

        if (!collisionCheck(checkNode)) { 
            return false;
        }
    }
    return true;
}


// ------------------- VISUALIZATION -----------------------------//

visualization_msgs::MarkerArray RRTStar::createBestPathMarker(std::vector<Point> points){

    visualization_msgs::MarkerArray markerArray;
    for (auto pt = 0; pt < points.size() - 1; pt++)
    {
        visualization_msgs::Marker marker;

        // We need to set the frame
        //  Set the frame ID and time stamp.
        marker.header.frame_id = "world";
        // single_marker_person.header.stamp = ros::Time();
        marker.header.stamp = ros::Time::now();

        // We set lifetime (it will dissapear in this many seconds)
        marker.lifetime = ros::Duration(0.2); // zero is forever

        // Set the namespace and id for this marker.  This serves to create a unique ID
        // Any marker sent with the same namespace and id will overwrite the old one
        marker.ns = "best_path"; // This is namespace, markers can be in diofferent namespace
        marker.id = ct_++;   // We need to keep incrementing markers to send others ... so THINK, where do you store a vaiable if you need to keep incrementing it

        // The marker type
        marker.type = visualization_msgs::Marker::LINE_STRIP;

        // Set the marker action.  Options are ADD and DELETE (we ADD it to the screen)
        marker.action = visualization_msgs::Marker::ADD;

        // Start point
        geometry_msgs::Point start;
        start.x = points.at(pt).x;
        start.y = points.at(pt).y;
        start.z = 0;  // or your node1.z if 3D
        marker.points.push_back(start);

        // End point
        geometry_msgs::Point end;
        end.x = points.at(pt + 1).x;
        end.y = points.at(pt + 1).y;
        end.z = 0;  // or your node2.z if 3D
        marker.points.push_back(end);

        // Orientation, we are not going to orientate it, for a quaternion it needs 0,0,0,1
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;

        marker.scale.x = 0.05;

        marker.color.r = 1;
        marker.color.g = 0;
        marker.color.b = 0;
        marker.color.a = 1.0;

        markerArray.markers.push_back(marker);
    }
    return markerArray;
}

visualization_msgs::MarkerArray RRTStar::createAvgPathMarker(std::vector<Point> points){

    visualization_msgs::MarkerArray markerArray;
    for (auto pt = 0; pt < points.size() - 1; pt++)
    {
        visualization_msgs::Marker marker;

        // We need to set the frame
        //  Set the frame ID and time stamp.
        marker.header.frame_id = "world";
        // single_marker_person.header.stamp = ros::Time();
        marker.header.stamp = ros::Time::now();

        // We set lifetime (it will dissapear in this many seconds)
        marker.lifetime = ros::Duration(0.2); // zero is forever

        // Set the namespace and id for this marker.  This serves to create a unique ID
        // Any marker sent with the same namespace and id will overwrite the old one
        marker.ns = "ave_path"; // This is namespace, markers can be in diofferent namespace
        marker.id = ct_++;   // We need to keep incrementing markers to send others ... so THINK, where do you store a vaiable if you need to keep incrementing it

        // The marker type
        marker.type = visualization_msgs::Marker::LINE_STRIP;

        // Set the marker action.  Options are ADD and DELETE (we ADD it to the screen)
        marker.action = visualization_msgs::Marker::ADD;

        // Start point
        geometry_msgs::Point start;
        start.x = points.at(pt).x;
        start.y = points.at(pt).y;
        start.z = 0;  // or your node1.z if 3D
        marker.points.push_back(start);

        // End point
        geometry_msgs::Point end;
        end.x = points.at(pt + 1).x;
        end.y = points.at(pt + 1).y;
        end.z = 0;  // or your node2.z if 3D
        marker.points.push_back(end);

        // Orientation, we are not going to orientate it, for a quaternion it needs 0,0,0,1
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;

        marker.scale.x = 0.08;

        marker.color.r = 1;
        marker.color.g = 1;
        marker.color.b = 0;
        marker.color.a = 1.0;

        markerArray.markers.push_back(marker);
    }
    return markerArray;
}

// Function to create marker for a leaf node
visualization_msgs::Marker RRTStar::createLeafNodeMarker(const Node& node) {
    
    visualization_msgs::Marker marker;
    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time::now();
    marker.ns = "leaf_node";
    marker.id = ct_++;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.lifetime = ros::Duration(0.2);

    marker.pose.position.x = node.x;
    marker.pose.position.y = node.y;
    marker.pose.position.z = 0;  // or your node.z if 3D

    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = 0.5;
    marker.scale.y = 0.5;
    marker.scale.z = 0.5;

    // Color (Red)
    marker.color.r = 1.0;
    marker.color.g = 0.5;
    marker.color.b = 0.0;
    marker.color.a = 1.0;  // Don't forget to set alpha

    return marker;
}

// Function to create marker for an edge
visualization_msgs::Marker RRTStar::createEdgeMarker(const Node& node1, const Node& node2) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time::now();
    marker.ns = "edge";
    marker.id = ct_++;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD;
    marker.lifetime = ros::Duration(0.2);

    // Start point
    geometry_msgs::Point start;
    start.x = node1.x;
    start.y = node1.y;
    start.z = 0;  // or your node1.z if 3D
    marker.points.push_back(start);

    // End point
    geometry_msgs::Point end;
    end.x = node2.x;
    end.y = node2.y;
    end.z = 0;  // or your node2.z if 3D
    marker.points.push_back(end);

    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    // Scale (width of the line)
    marker.scale.x = 0.02;

    // Color
    marker.color.r = 0.12;
    marker.color.g = 0.53;
    marker.color.b = 0.98;
    marker.color.a = 1.0;  // Don't forget to set alpha

    return marker;
}

// DEBUG
visualization_msgs::Marker RRTStar::createNodeMarker(const Node& node) {
    
    visualization_msgs::Marker marker;
    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time::now();
    marker.ns = "just_node";
    marker.id = ct_++;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.lifetime = ros::Duration(0.2);

    marker.pose.position.x = node.x;
    marker.pose.position.y = node.y;
    marker.pose.position.z = 0;  // or your node.z if 3D

    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;

    // Color (Red)
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;
    marker.color.a = 1.0;  // Don't forget to set alpha

    return marker;
}



visualization_msgs::MarkerArray RRTStar::createHeadpointMarker(const std::vector<Point>& points) {
        
    visualization_msgs::MarkerArray markerArray;
    const int numAhead = 5;

    for (int i = 0; i <= numAhead; i++){

        visualization_msgs::Marker marker;
        marker.header.frame_id = "world";
        marker.header.stamp = ros::Time::now();
        marker.ns = "headpoint";
        marker.id = ct_++;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.lifetime = ros::Duration(0.2);

        marker.pose.position.x = points.at(i).x;
        marker.pose.position.y = points.at(i).y;
        marker.pose.position.z = 0;  // or your node.z if 3D

        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;

        marker.scale.x = 0.5;
        marker.scale.y = 0.5;
        marker.scale.z = 0.5;

        // Color (Red)
        marker.color.r = 1.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;  // Don't forget to set alpha

        markerArray.markers.push_back(marker);
    }  
    return markerArray;  
}

void RRTStar::publishMarkers() {
    visualization_msgs::MarkerArray markerArray;

    // for (size_t i = 0; i < leafNodes_.size(); ++i) {
    //     Node node = leafNodes_.at(i);
    //     do {
    //         markerArray.markers.push_back(createEdgeMarker(node, nodeList_.at(node.parent)));
    //         node = nodeList_.at(node.parent);
    //     }
    //     while (node.parent != -1);
    // }

    // all branch
    // for (size_t i = 0; i < nodeList_.size(); ++i) {
    //     Node node = nodeList_.at(i);

    //     if (node.lastNode){
    //         do {
    //             markerArray.markers.push_back(createEdgeMarker(node, nodeList_.at(node.parent)));
    //             node = nodeList_.at(node.parent);
    //         }
    //         while (node.parent != -1);
    //     }
    // }

    // Create marker for each leaf node
    for (size_t i = 0; i < leafNodes_.size(); ++i) {
        const auto& leaf = leafNodes_.at(i);
        markerArray.markers.push_back(createLeafNodeMarker(leaf));
    }

    // Publish marker array
    markerPub_.publish(markerArray);
}

visualization_msgs::MarkerArray RRTStar::createConesCircleMarker(const std::vector<Point> cones)
{
    visualization_msgs::MarkerArray markerArray;
    const double radius = maxRangeFromTarget_;  // Circle radius

    for (int i = 0; i < cones.size(); i++)
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "world";
        marker.header.stamp = ros::Time::now();
        marker.ns = "headpoint";
        marker.id = ct_++;
        marker.type = visualization_msgs::Marker::LINE_STRIP;
        marker.action = visualization_msgs::Marker::ADD;
        marker.lifetime = ros::Duration(0.2);

        // Calculate circle points around the center point
        double center_x = cones.at(i).x;
        double center_y = cones.at(i).y;
        int numPoints = 20;  // Number of points to create a circle

        for (int j = 0; j <= numPoints; j++)
        {
            double angle = 2 * M_PI * (static_cast<double>(j) / numPoints);
            geometry_msgs::Point point;
            point.x = center_x + radius * cos(angle);
            point.y = center_y + radius * sin(angle);
            point.z = 0.0;  // Assuming a 2D plane

            marker.points.push_back(point);
        }

        // Close the circle by connecting the last point to the first point
        marker.points.push_back(marker.points.front());

        // Set other marker properties (e.g., color, scale, etc.)
        marker.scale.x = 0.04;  // Line width
        marker.color.r = 0.0;  // Red color
        marker.color.g = 1.0;  // Green color
        marker.color.b = 0.0;  // Blue color
        marker.color.a = 1;  // Opacity

        markerArray.markers.push_back(marker);
    }

    return markerArray;
}

visualization_msgs::MarkerArray RRTStar::createConesInsideCircleMarker(const std::vector<Point> cones)
{
    visualization_msgs::MarkerArray markerArray;
    const double radius = obstacleSize_;  // Circle radius

    for (int i = 0; i < cones.size(); i++)
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "world";
        marker.header.stamp = ros::Time::now();
        marker.ns = "headpoint";
        marker.id = ct_++;
        marker.type = visualization_msgs::Marker::LINE_STRIP;
        marker.action = visualization_msgs::Marker::ADD;
        marker.lifetime = ros::Duration(0.2);

        // Calculate circle points around the center point
        double center_x = cones.at(i).x;
        double center_y = cones.at(i).y;
        int numPoints = 20;  // Number of points to create a circle

        for (int j = 0; j <= numPoints; j++)
        {
            double angle = 2 * M_PI * (static_cast<double>(j) / numPoints);
            geometry_msgs::Point point;
            point.x = center_x + radius * cos(angle);
            point.y = center_y + radius * sin(angle);
            point.z = 0.0;  // Assuming a 2D plane

            marker.points.push_back(point);
        }

        // Close the circle by connecting the last point to the first point
        marker.points.push_back(marker.points.front());

        // Set other marker properties (e.g., color, scale, etc.)
        marker.scale.x = 0.04;  // Line width
        marker.color.r = 1.0;  // Red color
        marker.color.g = 0.0;  // Green color
        marker.color.b = 0.0;  // Blue color
        marker.color.a = 1;  // Opacity

        markerArray.markers.push_back(marker);
    }

    return markerArray;
}