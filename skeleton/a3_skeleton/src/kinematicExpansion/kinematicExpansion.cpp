#include "kinematicExpansion.h"
#include <visualization_msgs/MarkerArray.h>
#include <yaml-cpp/yaml.h>
#include <queue>
#include <unordered_set>



KE::KE(ros::NodeHandle& nh) : nh_{nh}, ct_{0}{
    markerPub_ = nh_.advertise<visualization_msgs::MarkerArray>("visualization_marker",1000,false);

    // Setting parameters
    std::string file_path = "/home/quanvu/git/pfms-2023a-minhquanvu0604/skeleton/a3_skeleton/config/KEparams.yaml";
    YAML::Node params = YAML::LoadFile(file_path);
    
    configNumToExpand_ = params["configNumToExpand"].as<double>(); 
    obstacleSize_ = params["obstacleSize"].as<double>();  
    expandDist_ = params["expandDist"].as<double>(); 
    expandAngle_ = params["expandAngle"].as<double>(); 
    expandAngle_ = expandAngle_ * M_PI / 180;
    maxDepth_ = params["maxDepth"].as<double>(); 
    // planDist_ = params["planDist"].as<double>(); 
    maxRangeFromCone_ = params["maxRangeFromCone"].as<double>(); 
    distToExclude_ = params["distToExclude"].as<double>(); 
    neighboorhoodRange_ = params["neighboorhoodRange"].as<double>(); 

    // std::cout << "----KE CONFIG----" << std::endl << std::endl;

    // std::cout << "configNumToExpand_: " << configNumToExpand_ << std::endl;
    // std::cout << "obstacleSize_: " << obstacleSize_ << std::endl;
    // std::cout << "expandDist_: " << expandDist_ << std::endl;
    // std::cout << "expandAngle_: " << expandAngle_ << std::endl;
    // std::cout << "expandAngle_ after conversion: " << expandAngle_ * M_PI / 180 << std::endl;

}

void KE::setInfo(const KNode& startNode, const std::vector<Point>& coneList){
    startNode_ = startNode;
    coneList_ = coneList;
}

// DEPTH FIRST STYLE
// void KE::createTree(){

//     ROS_INFO("TREE");

//     leafNodes_.clear();

//     std::shared_ptr<KNode> startNodePtr = std::make_shared<KNode>(startNode_);
//     expand(startNodePtr, 0);

//     markerPub_.publish(markerArray_);
//     markerArray_.markers.clear();
// }

// BREADTH FIRST STYLE
void KE::createTree(){
    ROS_INFO("TREE-0");
    
    nodeList_.clear();
    leafNodes_.clear();

    std::shared_ptr<KNode> startNodePtr = std::make_shared<KNode>(startNode_);

    // // exclude near cones - MAYBE NOT NEEDED
    // targetConeList_.clear();
    // for (auto cone : coneList_){
    //     double dist = euclideanDistance(startNode_.pose.position.x, startNode_.pose.position.y,cone.x,cone.y);
    //     if (dist > distToExclude_)
    //         targetConeList_.push_back(cone);
    // }
    // ROS_INFO_STREAM("targetConeList_ SIZE: " << targetConeList_.size());

    // queue for BFS
    std::queue<std::shared_ptr<KNode>> bfsQueue;
    bfsQueue.push(startNodePtr);

    ROS_INFO("TREE-1");

    while(!bfsQueue.empty()){

        auto currentNode = bfsQueue.front();
        bfsQueue.pop();
        expand(currentNode);
        
        for(auto & child : currentNode->children){

            // for (auto& nodePtr : nodeList_){
                
            //     double dist = euclideanDistance(child->pose.position.x,child->pose.position.y,nodePtr->pose.position.x,nodePtr->pose.position.y);
            //     ///////////////////////// REWIRING
            // }

            bfsQueue.push(child);
        }
    }

    // markerPub_.publish(markerArray_);
    // markerArray_.markers.clear()
    ROS_INFO("TREE-2");
    for (auto & node : nodeList_){
        if (node->depth == maxDepth_ - 1){
            leafNodes_.push_back(node);
            auto leafMarker = createLeafNodeMarker(node);
            markerArray_.markers.push_back(leafMarker);
        }
    }

        // if (node->reward){
        //     auto leafMarker = createJustNodeMarker(node);
        //     markerArray_.markers.push_back(leafMarker);
        // }

    ROS_INFO("TREE-3");
    // to find the best branch
    Branch bestBranch;
    // int maxBranchReward = -1;
    std::shared_ptr<KNode> bestLeafNode;

    for (auto leafNode : leafNodes_){

        std::shared_ptr<KNode> currentNode = leafNode;
        int branchReward = 0;

        do {
            branchReward += currentNode->reward;
            currentNode = currentNode->parent;
        }
        while (currentNode != nullptr);

        if (branchReward > bestBranch.reward){

            bestBranch.reward = branchReward;
            bestLeafNode = leafNode;
        }        
    }

    ROS_INFO("TREE-4");
    // go through the best branch
    std::shared_ptr<KNode> currentNode = bestLeafNode;
    
    if (currentNode == nullptr) {
        ROS_ERROR("bestLeafNode is nullptr");
        return;
    }

    do {
        bestBranch.path.push_back(currentNode);

        for (auto c : currentNode->conesNearNode) {
            auto it = std::find_if(bestBranch.conesNearBranch.begin(), bestBranch.conesNearBranch.end(),
                [&c](const std::shared_ptr<Point>& cone) { return *c == *cone; });

            if (it == bestBranch.conesNearBranch.end()) {
                // This cone is not in bestBranch.conesNearBranch, so add it
                bestBranch.conesNearBranch.push_back(c);
            }
        }

        auto leafMarker = createJustNodeMarker(currentNode);
        markerArray_.markers.push_back(leafMarker);

        if (currentNode->parent != nullptr) {
            currentNode = currentNode->parent;
        } else {
            break;
        }
    } while (currentNode != nullptr);

    ROS_WARN_STREAM("NODE NEAR BRANCH: " << bestBranch.conesNearBranch.size());

    // if (maxBranchReward < previousBranch_.reward){
    //     bestBranch = previousBranch_.path;
    // }



    // Get the goal
    // ROS_INFO_STREAM("best branch size: "<<bestBranch.size());

    if(bestBranch.path.size() >= 8) {

        visualization_msgs::Marker headpointMarker;
        // previous branch
        if (previousBranch_.reward != -1){
        
            ROS_WARN_STREAM("current reward: " << bestBranch.reward);
            ROS_WARN_STREAM("lase reward: " << previousBranch_.reward);
        
            if (bestBranch.reward + 10 < previousBranch_.reward) {

                bestBranch = previousBranch_;
                bestBranch.reward = previousBranch_.reward;  

                bestLeafNode = bestBranch.path.at(5);
                ROS_WARN_STREAM("use last branch");   
                headpointMarker = createHeadpointMarkerGreen(bestLeafNode);
            }
            else{
                bestLeafNode = bestBranch.path.at(7);
                headpointMarker = createHeadpointMarker(bestLeafNode);
                ROS_WARN_STREAM("use THIS branch"); 
            }
        }
            
        markerArray_.markers.push_back(headpointMarker);
        goal_ = bestLeafNode->pose.position;
        
    }
    else 
        return; //< Not updating goal_ -> use previous goal

    previousBranch_ = bestBranch;

    // goal_ = bestLeafNode->pose.position; 
    // std::cout << "GOAL: X=" << goal_.x << ", Y=" << goal_.y << std::endl;

    markerPub_.publish(markerArray_);
    markerArray_.markers.clear();
    traceBackAndDrawMarkers();

    // ROS_WARN_STREAM("num of nodes: " << nodeList_.size());
    ROS_INFO("TREE-5");
}

//void KE::expand(std::shared_ptr<KNode> node, int currentDepth = 0){
void KE::expand(std::shared_ptr<KNode> node){

    // Check for leaf node before generating children
    if (node->depth == maxDepth_){
        // leafNodes_.push_back(node);
        // auto leafMarker = createLeafNodeMarker(node);
        // markerArray_.markers.push_back(leafMarker);
        return; // If it's a leaf node, we're done expanding this branch
    }

    // ROS_INFO_STREAM("current depth: " << currentDepth);    

    for (int childBranch = -configNumToExpand_; childBranch <= configNumToExpand_; childBranch++){

        // ROS_INFO_STREAM("childBranch " << childBranch);
        
        // ROS_INFO("0");

        // if (currentDepth > MAXDEPTH) 
        //     return;

        // ROS_INFO("1");
        std::shared_ptr<KNode> child(new KNode());

        Point goalPoint;
        goalPoint.x = node->pose.position.x + expandDist_ * std::cos(node->pose.yaw + expandAngle_ * childBranch);
        goalPoint.y = node->pose.position.y + expandDist_ * std::sin(node->pose.yaw + expandAngle_ * childBranch);

        // check collision + node should not span too far from the nearest cone
        bool collision = false;
        double nearestDist = std::numeric_limits<double>::max();
        // int nearCount = 0;
        int coneRight = 0;
        int coneLeft = 0;

        // check all the cones
        std::vector<std::shared_ptr<Point>> conesNear;
        for (auto cone : coneList_){
            double distGoalToCone = euclideanDistance(goalPoint.x,goalPoint.y,cone.x,cone.y);

            if (distGoalToCone < obstacleSize_){
                collision = true;
                break;
            }

            // if (distGoalToCone < 15) // nearCount
            //     nearCount++;

            // EVLUATION FOR EACH NODE
            if (distGoalToCone < 7){ // 8

                std::shared_ptr<Point> conePtr = std::make_shared<Point>(cone);
                conesNear.push_back(conePtr);

                auto pt = globalToLocalFrame(cone,goalPoint,node->pose.yaw);
                if (pt.y > 0)
                    coneLeft++;
                else 
                    coneRight++;
            }

            if (distGoalToCone < nearestDist){
                nearestDist = distGoalToCone;
            }
        }
        if (collision) continue;
        
        if (coneLeft && coneRight){
            child->reward++;
            // auto markerNode = createLeafNodeMarker(child);
            // markerArray2_.markers.push_back(markerNode);
            // markerPub_.publish(markerArray2_);
            // markerArray2_.markers.clear();
            // ROS_ERROR("WOW");
        }

        double distFromCar = euclideanDistance(goalPoint.x,goalPoint.y,startNode_.pose.position.x,startNode_.pose.position.y);
        if (nearestDist > maxRangeFromCone_ && distFromCar > 8) continue;

        // ROS_INFO("2");

        Pose estimatedGoalPose;
        bool OK = checkOriginToDestination(node->pose,goalPoint,estimatedGoalPose);
        if (!OK)
            ROS_ERROR ("NOT FEASIBLE GOAL");

        // update child's data

        // double absTurning = child->absTurning + std::abs(estimatedGoalPose.yaw - node->pose.yaw);
        // ROS_INFO_STREAM("ABSTURNING: " << absTurning);
        // if (child->absTurning > M_PI/10){
        //     ROS_ERROR("OVER 60");
        //     continue;
        // }
        // child->absTurning = absTurning;


        // check with other nodes to eliminate them
        child->absTurning = node->absTurning + std::abs(childBranch); 

        bool badChild = false;
        for (auto it = nodeList_.begin(); it != nodeList_.end();){
            double dist = euclideanDistance(goalPoint.x,goalPoint.y,(*it)->pose.position.x,(*it)->pose.position.y);
            // ROS_INFO_STREAM("DIST: " << dist);
            if (dist > neighboorhoodRange_) 
                ++it; // Only increment iterator if no deletion is happening
            else{
                // ROS_ERROR("DELETE");
                if (child->absTurning > (*it)->absTurning){
                    badChild = true;
                    break;
                }
                else{
                    it = nodeList_.erase(it); // erase() returns an iterator pointing to the next element
                }
            }
        }
        if (badChild)
            continue;
        
        // accepted child
        child->pose = estimatedGoalPose;
        child->parent = node;
        child->depth = node->depth + 1;      
        child->conesNearNode = conesNear;
        
        // register the children to the parent
        node->children.push_back(child);
        // register to node list
        nodeList_.push_back(child);
        
        // // add visualization
        // auto edgeMarker = createEdgeMarker(node,child);
        // markerArray_.markers.push_back(edgeMarker);

        // ROS_INFO("3");
    }
}

Point KE::getGoal(){
    return goal_;
}

double KE::euclideanDistance(double x1, double y1, double x2, double y2){
    return std::pow(std::pow(x1 - x2,2) + std::pow(y1 - y2,2),0.5);
}

bool KE::checkOriginToDestination(Pose globalOrigin, Point globalGoal, Pose& estimatedGoalPose){
    Point localGoal = globalToLocalFrame(globalGoal,globalOrigin.position,globalOrigin.yaw);

    double linearDistanceToGoal = std::hypot(localGoal.x,localGoal.y);
    double alpha = atan2(localGoal.y, localGoal.x);
    double sweepingAngle = 2 * alpha; 
    double turningRadius = linearDistanceToGoal / (2 * sin(std::abs(alpha)));
   
    estimatedGoalPose.position.x = globalGoal.x;
    estimatedGoalPose.position.y = globalGoal.y;
    estimatedGoalPose.yaw = normalizeAngle(globalOrigin.yaw + sweepingAngle);

    double steeringAngle = atan2(WHEELBASE * 2 * sin(alpha),linearDistanceToGoal);

    // Check if less than the max steering angle allowed
    bool possible = std::abs(steeringAngle) < MAX_STEER_ANGLE;

    return possible;  
}

Point KE::globalToLocalFrame(Point goal, Point robotPosition, double robotYaw) {
    // Calculate the relative position of the goal in the global frame
    double dx = goal.x - robotPosition.x;
    double dy = goal.y - robotPosition.y;

    Point goalRobotFrame;
    
    // Convert the relative position to the robot frame using the robot's yaw angle
    goalRobotFrame.x = dx * cos(robotYaw) + dy * sin(robotYaw);
    goalRobotFrame.y = -dx * sin(robotYaw) + dy * cos(robotYaw);

    return goalRobotFrame;
}

double KE::normalizeAngle(double angle)
{
    angle = std::fmod(angle + M_PI, 2 * M_PI);
    if (angle < 0) {
        angle += 2 * M_PI;
    }
    return angle - M_PI;
}

void KE::traceBackAndDrawMarkers() {
    std::unordered_set<Edge> drawnEdges;

    // iterate through all leaf nodes
    for(auto& leaf : leafNodes_) {
        // create a shared_ptr to manipulate during tracing
        std::shared_ptr<KNode> current = leaf;

        // trace back from the leaf node to the root
        while(current->parent != nullptr) {
            Edge currentEdge = {current, current->parent};

            // check if the edge has already been drawn
            if(drawnEdges.find(currentEdge) == drawnEdges.end()) {
                // create marker for the edge between the current node and its parent
                auto edgeMarker = createEdgeMarker(current, current->parent);
                // add the edge marker to the marker array
                markerArray_.markers.push_back(edgeMarker);

                // add the edge to the set of drawn edges
                drawnEdges.insert(currentEdge);
            }
            
            // move to the parent
            current = current->parent;
        }
    }

    // publish all markers
    markerPub_.publish(markerArray_);
    markerArray_.markers.clear();
}



//-------------------------VISUALIZATION---------------------------//

visualization_msgs::Marker KE::createEdgeMarker(const std::shared_ptr<KNode> node1, const std::shared_ptr<KNode> node2) {
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
    start.x = node1->pose.position.x;
    start.y = node1->pose.position.y;
    start.z = 0;  // or your node1.z if 3D
    marker.points.push_back(start);

    // End point
    geometry_msgs::Point end;
    end.x = node2->pose.position.x;
    end.y = node2->pose.position.y;
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
    marker.color.a = 1.0; 

    return marker;
}


visualization_msgs::Marker KE::createNodeMarker(const std::shared_ptr<KNode> node) {
    
    visualization_msgs::Marker marker;
    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time::now();
    marker.ns = "just_node";
    marker.id = ct_++;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.lifetime = ros::Duration(0.2);

    marker.pose.position.x = node->pose.position.x;
    marker.pose.position.y = node->pose.position.y;
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
    marker.color.a = 1.0; 

    return marker;
}

visualization_msgs::Marker KE::createLeafNodeMarker(const std::shared_ptr<KNode> node) {
    
    visualization_msgs::Marker marker;
    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time::now();
    marker.ns = "leaf_node";
    marker.id = ct_++;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.lifetime = ros::Duration(0.2);

    marker.pose.position.x = node->pose.position.x;
    marker.pose.position.y = node->pose.position.y;
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

visualization_msgs::Marker KE::createJustNodeMarker(const std::shared_ptr<KNode> node) {
    
    visualization_msgs::Marker marker;
    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time::now();
    marker.ns = "leaf_node";
    marker.id = ct_++;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.lifetime = ros::Duration(0.2);

    marker.pose.position.x = node->pose.position.x;
    marker.pose.position.y = node->pose.position.y;
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
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;  // Don't forget to set alpha

    return marker;
}

visualization_msgs::Marker KE::createHeadpointMarker(const std::shared_ptr<KNode> node) {
    
    visualization_msgs::Marker marker;
    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time::now();
    marker.ns = "leaf_node";
    marker.id = ct_++;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.lifetime = ros::Duration(0.2);

    marker.pose.position.x = node->pose.position.x;
    marker.pose.position.y = node->pose.position.y;
    marker.pose.position.z = 0;  // or your node.z if 3D

    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = 0.9;
    marker.scale.y = 0.9;
    marker.scale.z = 0.9;

    // Color (Red)
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;
    marker.color.a = 1.0;  // Don't forget to set alpha

    return marker;
}

visualization_msgs::Marker KE::createHeadpointMarkerGreen(const std::shared_ptr<KNode> node) {
    
    visualization_msgs::Marker marker;
    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time::now();
    marker.ns = "leaf_node";
    marker.id = ct_++;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.lifetime = ros::Duration(0.2);

    marker.pose.position.x = node->pose.position.x;
    marker.pose.position.y = node->pose.position.y;
    marker.pose.position.z = 0;  // or your node.z if 3D

    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = 0.9;
    marker.scale.y = 0.9;
    marker.scale.z = 0.9;

    // Color (Red)
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;  // Don't forget to set alpha

    return marker;
}
