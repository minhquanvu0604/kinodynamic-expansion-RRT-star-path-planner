#include <iostream>
#include <vector>
#include <limits>
#include <cmath>
#include <stack>
#include <chrono>
#include <bitset>
#include <tuple>

#include "ackerman.h"
#include "quadcopter.h"
#include "pfms_types.h"
#include "supersearch.h"


/**
 * @brief Set goals
 * 
 * Set the goals for graph search if the argument is not an empty vector
 * 
 * @param goals Vector of Point for the goals
*/
void GraphSearch::setGoals(std::vector<pfms::geometry_msgs::Point> goals){
    
    if (!goals.empty())
        goals_ = goals;
}

/**
 * @brief Get minimum (most optimized) distance 
 * 
 * Result of the graph search to find the smallest distance to visit all the goals
 * Simply returns the minCost_ member variable, no further execution
*/
double GraphSearch::getMinCost(){
    return minCost_;
}


/**
 * @brief Graph search time elapsed 
 * 
 * Returns the time taken to conduct the graph search 
 * Only returns the timeElapsed_ member variable, no further execution
*/
double GraphSearch::getTimeElapsed(){
    return timeElapsed_;
}


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
std::vector<pfms::geometry_msgs::Point> GraphSearch::reorderPoint(const std::vector<int>& order, const std::vector<pfms::geometry_msgs::Point>& goalList){

    std::vector<pfms::geometry_msgs::Point> result(order.size());

    for (int i = 0; i < order.size(); i++){
        result.at(i) = goalList.at(order.at(i));
    }

    return result;
}


/**
 * @brief Distance between two points
 * 
 * Calculate the distance given the cartesian coordinate of two points. The parameters are two such Point objects
 * 
 * @param p1
 * @param p2
*/
double GraphSearch::distance(pfms::geometry_msgs::Point p1, pfms::geometry_msgs::Point p2) {
    return std::sqrt(std::pow(p1.x - p2.x, 2) + std::pow(p1.y - p2.y, 2));
}
