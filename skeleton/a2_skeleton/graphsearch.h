#ifndef GRAPHSEARCH_H
#define GRAPHSEARCH_H

#include <iostream>
#include <vector>
#include <limits>
#include <cmath>
#include <stack>
#include <chrono>

#include "pfms_types.h"
#include "controllerinterface.h"


/**
 * @brief Struct for Quadcopter graph search 
 * 
 * Group necessary variables to represent the state of Quacopter in all cases
*/
struct StateQuadcopter {
    int node; ///< The current node 
    double cost; ///< The current cost of travelling up to this node
    int nodesVisited; ///< Number of nodes visited
    std::vector<bool> visited; ///< The vector of bool indicating which nodes in the path vector has been visited
    std::vector<int> path; ///< The path proceeded up to the current node 
};

/**
 * @brief Struct for Quadcopter graph search 
 * 
 * Group necessary variables to represent the state of Ackerman in all cases
*/
struct StateAckerman {
    int node; ///< The current node 
    double cost; ///< The current cost of travelling up to this node
    int nodesVisited; ///< Number of nodes visited
    std::vector<bool> visited; ///< The vector of bool indicating which nodes in the path vector has been visited
    std::vector<int> path; ///< The path proceeded up to the current node 
    pfms::nav_msgs::Odometry currentOdo; ///< Current odometry, this is necessary for the checkOriginToDestination for the ackerman
};


/*!
 * \brief Graph Search Class
 * \details
 * This class implements graph search for the variations of the Travelling Salesman Problem. The class comprises searches for the
 * Quadcopter, Ackerman and SUPER MODE which combine the algorithm problem of the previous two. Each search 
 * is encapsulated in its own derived class.
 * \author Minh Quan Vu
 * \version 1.0.0
 * \date 2023-05-01
 * \pre none
 * \bug none reported as of 2023-05-02
*/

class GraphSearch
{
public:

    /**
     * @brief Set goals
     * 
     * @param goals Vector of Point for the goals
    */
    virtual void setGoals(std::vector<pfms::geometry_msgs::Point> goals);

    /**
     * @brief Graph search
     * 
     * Function to find the shortest path using backtracking and pruning with loops
    */
    virtual bool graphSearch() = 0;


    /**
     * @brief Get minimum (most optimized) distance 
     * 
     * Result of the graph search to find the smallest distance to visit all the goals
    */
    virtual double getMinCost();


    /**
     * @brief Graph search time elapsed 
     * 
     * Returns the time taken to conduct the graph search 
    */
    virtual double getTimeElapsed();


    /**
     * @brief Reorder the point according the vector of integer
     * 
     * Used to find the resulting goal Point order after finding the optimal order of visiting goals as 
     * vector onf integers
     * 
     * @param order Reference to constant vector of integers denoting the optimal order of goals
     * @param goalList Reference to constant vector of Points denoting the initial order of goals. The vector of integers
     * is the index of the optimal Point in reference to goalList
    */
    std::vector<pfms::geometry_msgs::Point> reorderPoint(const std::vector<int>& order, const std::vector<pfms::geometry_msgs::Point>& goalList);

protected:

    /// @brief Vector of goals used to conduct graph search
    std::vector<pfms::geometry_msgs::Point> goals_;

    /// @brief The smallest path found by the graph search
    double minCost_;

    /// @brief The time elapsed of executing the graph search
    double timeElapsed_;

    /**
     * @brief Distance between two points
     * 
     * Calculate the distance given the cartesian coordinate of two points. The parameters are two such Point objects
     * 
     * @param p1
     * @param p2
    */
    double distance(pfms::geometry_msgs::Point p1, pfms::geometry_msgs::Point p2);
};


#endif
