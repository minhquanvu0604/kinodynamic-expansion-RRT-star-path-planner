#include <iostream>
#include <vector>
#include <limits>
#include <cmath>
#include <stack>
#include <chrono>

#include "quadcopter.h"
#include "pfms_types.h"
#include "quadcoptersearch.h"


/**
 * @brief Default constructor
 * 
 * The default constructors set necessary member variables to a default value 
*/
QuadcopterSearch::QuadcopterSearch() : quadcopterPtr_{nullptr}{}


/**
 * @brief Overloaded constructor
 * 
 * The overloaded constructors takes in the pointer to ControllerInterface object, in this context it is a pointer to 
 * Quadcopter object and is passed to the member variable
 * 
 * @param quadcopterPtr pointer to ControllerInterface object, which in this context is the Quadcopter
*/
QuadcopterSearch::QuadcopterSearch(ControllerInterface* quadcopterPtr) : quadcopterPtr_{quadcopterPtr}{}


/**
 * @brief Set pointer to ControllerInterface object
 * 
 * This is necessary to conduct graphsearch and is used only when the object is constructed with default contrustor
 * 
 * @param quadcopterPtr pointer to ControllerInterface object, which in this context is the Quadcopter
*/
void QuadcopterSearch::setPtr(ControllerInterface* quadcopterPtr){
    quadcopterPtr_ = quadcopterPtr;
}


/**
 * @brief Implementing Ackerman graph search and set member variables regarding the optimal path order (vector of inteter)
 * and the minimum distance found
 * 
 * The graph search method used here is an optimization from the brute force method, utilizing backtracking and pruning
 * to avoid unnecessary computation. It utilize the stack data type for fast popping and pushing. Every state is denoted 
 * by an object of StateAckerman struct which consisting of all the information it needs for complete representation of state.
 * We visit nodes one to one, visiting all the possible neighboors and record as new states popped into the stack. 
 * Every time the distance travelled is larger than the current minimum, the action of visiting to neighboors is skipped and 
 * we safely eliminate the whole following branch. This is better than the brute force method where all permutations have to
 * be evaluated
 * 
 * For the Quadcopter search, it is closer to the Travelling Salesman Problem as all the goals are accessible at any state, and the 
 * path between goals are linear path. However, it is crucial to recognize that the initial position of the Quadcopter is 
 * not included in the goal list, but shoulbe be taken into account as different positions of the Quadcopter can yield different 
 * optimal resulting path and goal order. To cope with this, we get the odometry of the starting point of the Quacopter and insert it 
 * into the vector of goals. After the processing is complete, the initial position shoul also be the first point to visit, given that
 * the algorithm computes correctly. Finally, we erase that starting point out to yield the best order of goals with the same size as
 * the goal list
*/
bool QuadcopterSearch::graphSearch(){

    // For runtime analysis
    auto start_time = std::chrono::high_resolution_clock::now();

    if (quadcopterPtr_ == nullptr) 
        return false;

    if (goals_.empty()) 
        return true;

    // Get initial odometry
    auto initialOdo = quadcopterPtr_->getOdometry();

    pfms::geometry_msgs::Point initialPosition = {initialOdo.position.x, initialOdo.position.y, 0};
    goals_.insert(goals_.begin(), initialPosition);
    
    // Create a distance matrix
    std::vector<std::vector<double>> dist(goals_.size(), std::vector<double>(goals_.size(), 0));
    for (int i = 0; i < goals_.size(); i++) {
        for (int j = 0; j < goals_.size(); j++) {
            if (i != j) {
                dist.at(i).at(j) = distance(goals_.at(i), goals_.at(j));
            }
        }
    }

    // Find the best path and the corresponding minimum path length
    double minCost = std::numeric_limits<double>::max();

    // Vector of int to store the best path 
    std::vector<int> bestPath;

    // A stack data type to optimally store the states when utilizing backtracking    
    std::stack<StateQuadcopter> st;

    // Push the initial state into the stack
    st.push({0, 0, 1, std::vector<bool>(goals_.size(), false), std::vector<int>(1, 0)});

    while (!st.empty()) {

        // We process with the state at the top of the stack and pop with out as already processed
        StateQuadcopter current = st.top();
        st.pop();

        // Update current node as visited
        current.visited.at(current.node) = true;

        // Whether all the nodes have been visited
        if (current.nodesVisited == goals_.size()) {

            // Save the min cost and the best path up to the current processing
            if (current.cost < minCost) {
                minCost = current.cost;
                bestPath = current.path;
            }
            continue;
        }

        // Go through all the neighbors of the current node
        for (int i = 0; i < goals_.size(); i++) {
            if (!current.visited.at(i) && dist.at(current.node).at(i) > 0) {
                double currCost = current.cost + dist.at(current.node).at(i);

                // Pruning
                if (currCost < minCost) {

                    // Construct a new state with the updated path and push it into the stack
                    std::vector<int> newPath = current.path;
                    newPath.push_back(i);
                    st.push({i, currCost, current.nodesVisited + 1, current.visited, newPath});
                }
            }
        }
    }

    // Delete the first element of the bestPath and goalsQuadcopter as it is the initial position not a goal
    bestPath.erase(bestPath.begin());
    for (auto& element : bestPath) 
        element--;
    goals_.erase(goals_.begin());

    
    // Update results into member variables
    minCost_ = minCost;
    bestPath_ = bestPath;
    // if (bestPath.size() != goals_.size()) 

    // Stop the timer
    auto end_time = std::chrono::high_resolution_clock::now();

    // Calculate the duration in seconds 
    timeElapsed_ = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();

    return true;
}


/**
 * @brief Get the best path
 * 
 * Return the best path order as a vector of integers
*/
std::vector<int> QuadcopterSearch::getBestPath(){
    return bestPath_;
}