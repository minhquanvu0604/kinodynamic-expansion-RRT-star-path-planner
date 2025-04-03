#ifndef KINEMATIC_EXPANSION_H
#define KINEMATIC_EXPANSION_H

#include <vector>
#include <visualization_msgs/MarkerArray.h>
#include <memory>

#include "ros/ros.h"
#include "heuristicRRTstar/heuristicRRTstar.h"
#include "ackerman.h"


struct Pose{
    Point position;
    double yaw;
};

struct KNode {
    Pose pose;
    double depth = 0;
    double absTurning = 0;
    double reward =0;
    
    std::vector<std::shared_ptr<KNode>> children;
    std::shared_ptr<KNode> parent;

    std::vector<std::shared_ptr<Point>> conesNearNode;
};

struct Branch {
    std::vector<std::shared_ptr<KNode>> path;
    double reward = -1;

    std::vector<std::shared_ptr<Point>> conesNearBranch;

    Branch(){};
    Branch(const std::vector<std::shared_ptr<KNode>>& path, double reward)
        : path(path), reward(reward) {}
    // Branch& operator=(const Branch& other) {
    // if (this != &other) {
    //     path = other.path;
    //     reward = other.reward;
    // }
    // return *this;
    // }
};

struct Edge {
    std::shared_ptr<KNode> node1;
    std::shared_ptr<KNode> node2;

    // override the equality operator to allow easy comparison of edges
    bool operator==(const Edge& other) const {
        // an edge is the same if it contains the same nodes, regardless of the order
        return (node1 == other.node1 && node2 == other.node2) || (node1 == other.node2 && node2 == other.node1);
    }
};

// provide a specialization of std::hash for Edge
namespace std {
    template<>
    struct hash<Edge> {
        std::size_t operator()(const Edge& edge) const {
            // combine the hashes of the two nodes
            return std::hash<std::shared_ptr<KNode>>()(edge.node1) ^ std::hash<std::shared_ptr<KNode>>()(edge.node2);
        }
    };
}


class KE {
    
public:    
    KE(ros::NodeHandle& nh);
    
    void setInfo(const KNode& startNode, const std::vector<Point>& coneList);

    // void expand(std::shared_ptr<KNode> node, int currentDepth);
    void expand(std::shared_ptr<KNode> node);

    void createTree();

    void traceBackAndDrawMarkers();

    Point getGoal();



private:
    KNode startNode_;

    //double angleResolution_; // Even num
    int configNumToExpand_;
    double obstacleSize_;

    double expandDist_;
    double expandAngle_;
    int maxDepth_;
    double maxRangeFromCone_;
    double distToExclude_;
    double neighboorhoodRange_;

    // double planDist_;    

    std::vector<Point> coneList_;
    // std::vector<Point> targetConeList_;

    std::vector<std::shared_ptr<KNode>> nodeList_;
    std::vector<std::shared_ptr<KNode>> leafNodes_;

    visualization_msgs::MarkerArray markerArray_;

    Point goal_;

    // std::vector<std::shared_ptr<KNode>> bestBranch_;
    Branch previousBranch_;


    double euclideanDistance(double x1, double y1, double x2, double y2);
    bool checkOriginToDestination(Pose globalOrigin, Point globalGoal, Pose& estimatedGoalPose);
    Point globalToLocalFrame(Point goal, Point robotPosition, double robotYaw);
    double normalizeAngle(double angle);

    // Visualization
    visualization_msgs::Marker createEdgeMarker(const std::shared_ptr<KNode> node1, const std::shared_ptr<KNode> node2);
    visualization_msgs::Marker createNodeMarker(const std::shared_ptr<KNode> node);
    visualization_msgs::Marker createLeafNodeMarker(const std::shared_ptr<KNode> node);
    visualization_msgs::Marker createJustNodeMarker(const std::shared_ptr<KNode> node);
    visualization_msgs::Marker createHeadpointMarker(const std::shared_ptr<KNode> node);
    visualization_msgs::Marker createHeadpointMarkerGreen(const std::shared_ptr<KNode> node);




    static constexpr double STEERING_RATIO = 17.3;
    static constexpr double LOCK_TO_LOCK_REVS = 3.2;
    static constexpr double MAX_STEER_ANGLE = (M_PI*LOCK_TO_LOCK_REVS/STEERING_RATIO); // 0.581104 ~ 33.294806658 degree
    // Steer angle negative -> turn right
    // Steer angle positive -> turn left
    static constexpr double WHEELBASE = 2.65; 

    static constexpr int MAXDEPTH = 1000;

private: 
    
    ros::NodeHandle& nh_; //< ROS node handle
    long ct_; //< Counter for marker
    
    ros::Publisher markerPub_; //< ROS publisher object
};


#endif