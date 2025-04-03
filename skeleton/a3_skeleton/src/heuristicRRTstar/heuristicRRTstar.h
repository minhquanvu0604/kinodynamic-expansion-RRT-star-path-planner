#ifndef HEURISTIC_RRTSTAR_H
#define HEURISTIC_RRTSTAR_H

#include <vector>
#include <random>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>
#include <deque>

#include "ros/ros.h"


/*! @brief Node data type
*/
struct Node {
    // State presentation

    double x; 
    double y;
    double cost;
    int parent = -1;
    std::vector<int> children;

    // Evaluation

    // double turning = -1; //< Sum of absolute value of angle turned of each node
    double yaw = -1; //< Global orientation 

    // Check if two nodes have the same coordinate, used for the find() function in @sa createTree
    bool operator==(const Node& other) const {
        return x == other.x && y == other.y; 
    }
};

/*! @brief Point data type
*/
struct Point {
    double x;
    double y;

    Point() : x(0), y(0) {} 
    Point(double xValue, double yValue) : x(xValue), y(yValue) {}

    // Comparator for sorting
    bool operator < (const Point &p) const {
        return x < p.x || (x == p.x && y < p.y);
    }
    
    // equality comparator, with tolerance = 3
    bool operator==(const Point &p) const {
        return std::abs(x - p.x) < 3 && std::abs(y - p.y) < 3;
    }
};


class RRTStar {
public:
    
    /*! @brief Default constructor
    */
    RRTStar();

    /*! @brief Constructor that allocates internals
     *
     * @param[in]    nh - ROS nodehandle
     * @param[in|out]    id - used for creating markers, avoiding markers having the same id, take reference 
     * random spawn of points and the evaluation of node confidence reward
    */
    RRTStar(ros::NodeHandle& nh, long& id);


    /*! @brief Constructor that allocates internals
     *
     * @param[in]    nh - ROS nodehandle
     * @param[in|out]    id - used for creating markers, avoiding markers having the same id, take reference 
     * @param[in]    start - start node
     * @param[in]    obstacleList - vector of point representing the cones
     * @param[in]    rrtConeTargets - vector of point representing the cones that will be used to applied certain operation for the tree, such as the
     * random spawn of points and the evaluation of node confidence reward
    */
    RRTStar(ros::NodeHandle& nh, long& id, Node start, const std::vector<Point>& obstacleList, const std::vector<Point>& rrtConeTargets);


    /*! @brief Constructor that allocates internals
     *
     * @param[in]    nh - ROS nodehandle
     * @param[in|out]    id - used for creating markers, avoiding markers having the same id, take reference 
     * @param[in]    start - start node
     * @param[in]    iteration - number of random node spawned for the tree search, the larger the for computation, the more ideal path
     * @param[in]    planDistance - max distance for a branch of the tree, the last node will be recorded as leaf node, only branches with leaf node 
     * will be consider for the graph search
     * @param[in]    turnAngle - angle contrained for the new random node, the new node will not span further from this angle
     * @param[in]    neighboorhoodRange - the range used for rewiring neighboor node, which is a feature of RRT*
     * @param[in]    expandDist - distance fixed for the new node to spawn from the existing one
     * @param[in]    animation - whether to visualize the tree
     * @param[in]    obstacleSize - size of obstacle as a circle around the cones, this value specifies the circle radius
     * @param[in]    maxRangeFromTarget - random point will be spawned around a limited distance from the cones, this value sets this distance
     * @param[in]    obstacleList - vector of point representing the cones
     * @param[in]    rrtConeTargets - vector of point representing the cones that will be used to applied certain operation for the tree, such as the
     * random spawn of points and the evaluation of node confidence reward
    */
    RRTStar(ros::NodeHandle& nh, long& id, Node start, double iteration, double planDistance, double turnAngle, double neighboorhoodRange,
                double expandDist, bool animation, double obstacleSize, double maxRangeFromTarget, double distanceToExclude,
                const std::vector<Point>& obstacleList, const std::vector<Point>& rrtConeTargets);


    /*! @brief Setter for some data 
     *
     * @param[in]    start - start node
     * @param[in]    obstacleList - vector of point representing the cones
     * @param[in]    rrtConeTargets - vector of point representing the cones that will be used to applied certain operation for the tree, such as the
    */
    void setData(Node start, const std::vector<Point>& obstacleList, const std::vector<Point>& rrtConeTargets);


    /*! @brief Create the tree
     *
     * The main function to create the tree, will modify nodeList_ and leafNodes_ member varibles
    */
    void createTree();


    /*! @brief Getter for the list of nodes created by the tree
     *
     * @param[out]  - vector of nodes
    */
    std::vector<Node> getNodeList();


    /*! @brief Getter for the list of leaf nodes created by the tree
     *
     * @param[out]  - vector of leafnodes
    */
    std::vector<Node> getLeafNodes();


    /*! @brief Choose the best path out of all the paths created by the tree
     *
     * @param[out]  - vector of points in near-to-far order, representing the best path
    */
    std::vector<Point> chooseBestPath();


    /*! @brief Calculate the average path from a number of previous best paths
     *
     * @param[in]   paths - a deque of vectors of points representing previous paths
     *
     * @param[out]  - vector of points in near-to-far order, representing the moving average of the previous best paths
    */
    std::vector<Point> calculateMovingAveragePath(const std::deque<std::vector<Point>>& paths);


    /*! @brief Update the deque of previous paths
     *
     * @param[in|out]   paths - a deque of vectors of points representing previous paths
     * @param[in]   newPaths - most recent best path produced from the tree
     *
    */
    void updatePaths(std::deque<std::vector<Point>>& paths, const std::vector<Point>& newPath);


private:

    ros::NodeHandle& nh_; //< ROS node handle
    long& id_; //< Reference to counter for marker
    long ct_; //< Counter for marker
    Node start_; //< Start node
    std::vector<Point> obstacleList_; //< List of ostacles
    std::vector<Point> rrtConeTargets_; //< List of targets for RRT* tree search

    double iteration_; //< start node
    double planDistance_; //< number of random node spawned for the tree search, the larger the for computation, the more ideal path
    double turnAngle_; //< angle contrained for the new random node, the new node will not span further from this angle
    double expandDist_; //< distance fixed for the new node to spawn from the existing one
    bool animation_ ; //< whether to visualize the tree
    double obstacleSize_; //< size of obstacle as a circle around the cones, this value specifies the circle radius
    double maxRangeFromTarget_; //< random point will be spawned around a limited distance from the cones, this value sets this distance
    double neighboorhoodRange_; //< the range used for rewiring neighboor node, which is a feature of RRT*
    double distanceToExclude_ = 10; //< Distance used to exclude points from the obstacle list to produce RRT* target list 

    // Result of RRTStar
    std::vector<Node> nodeList_; //< List of nodes created by the tree
    std::vector<Node> leafNodes_; //< List of leaf nodes created by the tree

    std::mt19937 gen_; //< Agent for generating random numbers

    ros::Publisher markerPub_; //< ROS publisher object


    /*! @brief Get a random point
     *
     * @param[out]  - random point
    */
    Point getRandomPoint();


    /*! @brief Get a random point from the RRT* target list
     *
     * @param[out]  - random point
    */
    Point getRandomPointFromTargetList();


    /*! @brief Get a the index of the nearest node for the new node
     *
     * @param[out]  - index of the nearest node for the new node
    */
    int getNearestListIndex(const std::vector<Node>& nodeList, const Point& rnd);


     /*! @brief Apply angular constraints and return an appropriate node
     * 
     * @param[in]   randomPoint - the random point to inspect
     * @param[in]   nearestIndex - index of the nearest node for the new node
     * @param[out]  - node after applying constraints
    */
    Node angularConstraint(const Point& randomPoint, int nearestIndex);


    /*! @brief Find the neighboor of this node, range restricted by neighboorhoodRange_
     * 
     * @param[in]    node - node to find neighboors
     * @param[out]  - vector of indices of the random points
    */
    std::vector<int> findNeighboors(const Node& newNode);
    

    /*! @brief Take reference to new node and update its parent and cost
     *
     * @param[in]    newNode - node to find its parent
     * @param[in]    nearIndices - list of indices of neighboor node found by @sa findNeighboors
    */
    void chooseParent(Node& newNode, std::vector<int> nearIndices);


    /*! @brief Rewire the other nodes from the neighboor hood to improve their paths
     *
     * The other neighboors will be inspected as whether will be the children node of the newNode
     *
     * @param[in]    newNode - node as potential parent for the other neighboor nodes
     * @param[in]    nearIndices - list of indices of neighboor node found by @sa findNeighboors
    */
    void rewire(Node& newNode, int newNodeInd, const std::vector<int>& nearInds);


    /*! @brief Convert from global frame to robot's local frame
     *
     * @param[in]    goal - point indicating the target 
     * @param[in]    robot - Node data type representing the robot
    */
    Point globalToLocalFrame(Point goal, Node robot);


    /*! @brief Set the RRT* cone target list
    */
    void setTargetCone();

    /*! @brief Check for collision with the obstacle
     * 
     * A premature check that consider only 2 ends points not the whole line
     * 
     * @param[in]    node - node to inspect collision
     * @param[out] - true indicating no collision
    */
    bool collisionCheck(const Node& node);

    /*! @brief Check for collision with the obstacle
     * 
     * An extension version of collision check
     * 
     * @param[in]    node - node to inspect collision
     * @param[out] - true indicating no collision
    */
    bool isPathFree(const Node& newNode, const Node& nearestNode);


    /*! @brief Check which side the cone is in
     *      
     * @param[in]    node - node to inspect collision
     * @param[in]    cone - the point representing the cone
     * @param[out]   bool indicating if the cone is on the left of the node
    */
    bool isConeLeftFromNode(Node node, Point cone);


    /*! @brief Normalize angle
     *  Normalize the angle to span within -pi to pi
    */
    double normalizeAngle(double angle);

    /*! @brief Calculate the euclidean distance between 2 points
    */
    double euclideanDistance(double x1, double y1, double x2, double y2);
    

    /*! @brief Calculate the angle between 2 vectors
     * 
     * The inputs are x and y coordinate of the end points and head points of 2 vectors respectively
    */
    double calculateAngle(double x1, double y1, double x2, double y2, double x3, double y3, double x4, double y4);

    
    //--------------------- MARKERS ---------------------------------------------------------//
    /*! @brief Create edge marker */
    visualization_msgs::Marker createEdgeMarker(const Node& node1, const Node& node2);

    /*! @brief Create leaf node marker */
    visualization_msgs::Marker createLeafNodeMarker(const Node& node);

    /*! @brief Publish edge and leaf node marker */
    void publishMarkers();

    visualization_msgs::Marker createNodeMarker(const Node& node);



    /*! @brief Create best path marker */
    visualization_msgs::MarkerArray createBestPathMarker(std::vector<Point> points);

    /*! @brief Create average path marker */    
    visualization_msgs::MarkerArray createAvgPathMarker(std::vector<Point> points);

    /*! @brief Create head point marker */   
    visualization_msgs::MarkerArray createHeadpointMarker(const std::vector<Point>& points);

public:
    visualization_msgs::MarkerArray createConesCircleMarker(std::vector<Point> cones);

    visualization_msgs::MarkerArray createConesInsideCircleMarker(const std::vector<Point> cones);

    //--------------------- MARKERS ---------------------------------------------------------//
};


/*! @brief Convert from Point data type to ROS's geometry_msgs/Point
*/
inline geometry_msgs::Point convertToPointMsg(Point& point) {
    geometry_msgs::Point pointMsg;
    pointMsg.x = point.x;
    pointMsg.y = point.y;
    pointMsg.z = 0.0;
    return pointMsg;
}


/*! @brief Convert from ROS's geometry_msgs/Point to Point data type  
*/
inline Point convertToPoint(const geometry_msgs::Point& pointMsg) {
    Point point;
    point.x = pointMsg.x;
    point.y = pointMsg.y;
    return point;
}


/*! @brief Convert from a vector of Point data type to a vector of ROS's geometry_msgs/Point
*/
inline std::vector<geometry_msgs::Point> convertToGeometryPointVector(const std::vector<Point>& points) {
    std::vector<geometry_msgs::Point> geometryPoints;
    geometryPoints.reserve(points.size());

    for (const auto& point : points) {
        geometry_msgs::Point geometryPoint;
        geometryPoint.x = point.x;
        geometryPoint.y = point.y;
        geometryPoint.z = 0.0;
        geometryPoints.push_back(geometryPoint);
    }

    return geometryPoints;
}

/*! @brief Convert from a vector of ROS's geometry_msgs/Point to a vecotr of Point data type  
*/
inline std::vector<Point> convertToPointVector(const std::vector<geometry_msgs::Point>& geometryPoints) {
    std::vector<Point> points;
    points.reserve(geometryPoints.size());

    for (const auto& geometryPoint : geometryPoints) {
        Point point(geometryPoint.x, geometryPoint.y);
        points.push_back(point);
    }

    return points;
}

#endif 

