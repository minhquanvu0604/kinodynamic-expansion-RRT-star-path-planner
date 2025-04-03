#include <iostream>
#include <vector>
#include <limits>
#include <cmath>
#include <stack>
#include <chrono>
#include <bitset>
#include <tuple>

#include "ackerman.h"
#include "pfms_types.h"
#include "ackermansearch.h"

/**
 * @brief Default constructor
 * 
 * The default constructors set necessary member variables to a default value 
 * The ackermanPtr_ is set to nullptr by default as a practive to avoid undefined behavior
*/
AckermanSearch::AckermanSearch() : ackermanPtr_{nullptr}{}


/**
 * @brief Overloaded constructor
 * 
 * The overloaded constructors takes in the pointer to ControllerInterface object, in this context it is a pointer to 
 * Ackerman object and is passed to the member variable
 * 
 * @param quadcopterPtr pointer to ControllerInterface object, which in this context is the Ackerman
*/
AckermanSearch::AckermanSearch(ControllerInterface* ackermanPtr) : ackermanPtr_{ackermanPtr}{}


/**
 * @brief Set pointer to ControllerInterface object
 * 
 * This is necessary to conduct graphsearch and is used only when the object is constructed with default contrustor
 * 
 * @param quadcopterPtr pointer to ControllerInterface object, which in this context is the Ackerman
*/
void AckermanSearch::setPtr(ControllerInterface* ackermanPtr){
    ackermanPtr_ = ackermanPtr;
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
 * be evaluated. 
 * 
 * For the Ackerman search, it is necessary that we only inspects goals that are reachable, so throughout the visiting of 
 * neighboors, the @sa checkOriginToDestination is called to eliminate neighboors that are unviable to visit 
*/
bool AckermanSearch::graphSearch(){

    // For runtime analysis
    auto start_time = std::chrono::high_resolution_clock::now();

    if (ackermanPtr_ == nullptr) 
        return false;

    if (goals_.empty()) 
        return true;

    bool foundAny = false;

    // Get initial odometry
    auto initialOdo = ackermanPtr_->getOdometry();

    // Find the best path and the corresponding minimum path length
    double minCost = std::numeric_limits<double>::max();

    // Vector of int to store the best path 
    std::vector<int> bestPath;

    // A stack data type to optimally store the states when utilizing backtracking 
    std::stack<StateAckerman> st;
    
    // Push the initial state into the stack
    st.push({0, 0, 1, std::vector<bool>(goals_.size(), false), std::vector<int>(1, 0), initialOdo});
    
    pfms::nav_msgs::Odometry estimatedGoalPose;
    
    while (!st.empty()) {

        // We process with the state at the top of the stack and pop with out as already processed
        StateAckerman current = st.top();
        st.pop();

        // Update current node as visited
        current.visited.at(current.node) = true;

        // Whether all the nodes have been visited
        if (current.nodesVisited == goals_.size()) {
            foundAny = true;    

            // Save the min cost and the best path up to the current processing
            if (current.cost < minCost) {
                minCost = current.cost;
                bestPath = current.path;
            }
            continue;
        }

        // Go through all the neighbors of the current node
        for (int i = 0; i < goals_.size(); i++) {
            if (current.node == i) continue;

            double distanceToNext;
            double dummyTimeToNext;

            // If the next goal is no reachable, set the distance to -1
            bool OK = ackermanPtr_->checkOriginToDestination(current.currentOdo, goals_.at(i), distanceToNext, dummyTimeToNext, estimatedGoalPose);
            if (!OK) {
                distanceToNext = -1;
            }

            // Go through all the neighbors of the current node, skip if the current node is not reachable
            if (!current.visited.at(i) && distanceToNext > 0) {

                double currCost = current.cost + distanceToNext;

                // Pruning
                if (currCost < minCost) {

                    // Construct a new state with the updated path and push it into the stack
                    std::vector<int> newPath = current.path;
                    newPath.push_back(i);
                    st.push({i, currCost, current.nodesVisited + 1, current.visited, newPath, estimatedGoalPose});
                }
            }
        }
    }

    // Update results into member variables
    minCost_ = minCost;
    bestPath_ = bestPath;
    
    // Stop the timer
    auto end_time = std::chrono::high_resolution_clock::now();

    // Calculate the duration in seconds 
    timeElapsed_ = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();

    return foundAny;
}


/**
 * @brief Get the best path
 * 
 * Return the best path order as a vector of integers
*/
std::vector<int> AckermanSearch::getBestPath(){
    return bestPath_;
}