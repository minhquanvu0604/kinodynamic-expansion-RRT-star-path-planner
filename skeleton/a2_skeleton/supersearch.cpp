#include <iostream>
#include <vector>
#include <limits>
#include <cmath>
#include <stack>
#include <chrono>

#include "pfms_types.h"
#include "supersearch.h"
#include "quadcoptersearch.h"
#include "ackermansearch.h"


/**
 * @brief Default constructor
 * 
 * The default constructors set necessary member variables to a default value 
*/
SuperSearch::SuperSearch(){}



SuperSearch::SuperSearch(ControllerInterface* ackermanPtr, ControllerInterface* quadcopterPtr)
                            : ackermanPtr_{ackermanPtr}, quadcopterPtr_{quadcopterPtr}{}

void SuperSearch::setPtrs(ControllerInterface* ackermanPtr, ControllerInterface* quadcopterPtr){
    ackermanPtr_ = ackermanPtr;
    quadcopterPtr_ = quadcopterPtr;
}

void SuperSearch::setGoals(std::vector<pfms::geometry_msgs::Point> goals){
    goals_.insert(goals_.end(), goals.begin(), goals.end());
}

bool SuperSearch::graphSearch(){

    // For runtime analysis
    auto start_time = std::chrono::high_resolution_clock::now();

    // Boolean indicating whether a valid permuation is found
    bool foundAny = false;

    // Setting up objects and necessary member variables
    AckermanSearch ackerSearch(ackermanPtr_);
    QuadcopterSearch quadSearch(quadcopterPtr_);

    // Assignment vector of boolean, false -> ackerman, true -> quadcopter
    // All the goals are in order with goals
    std::vector<bool> assignment(goals_.size(), false);

    // Store the min distance
    double minDist = std::numeric_limits<double>::max();

    // Store the best order for each platforms, the index corresponds to assignment vector above
    std::vector<int> bestOrderAckerman;
    std::vector<int> bestOrderQuadcopter;

    // Run the loop through all permutations
    do {
        // Store the goals of the current permutation ("local") 
        std::vector<pfms::geometry_msgs::Point> localGoalsAckerman;
        std::vector<pfms::geometry_msgs::Point> localGoalsQuadcopter;
        
        // Store the order from the local goals
        std::vector<int> localOrderAckerman; 
        std::vector<int> localOrderQuadcopter;   

        for (int i = 0; i < assignment.size(); i++){
            if (!assignment.at(i)){ // Ackerman
                localGoalsAckerman.push_back(goals_.at(i));
                localOrderAckerman.push_back(i);
            }

            else if (assignment.at(i)){ // Quadcopter
                localGoalsQuadcopter.push_back(goals_.at(i));
                localOrderQuadcopter.push_back(i);
            }
        }

        // Implement graph search
        ackerSearch.setGoals(localGoalsAckerman);
        quadSearch.setGoals(localGoalsQuadcopter);

        bool OK = ackerSearch.graphSearch();
        if (!OK) continue; // This permutation is not possible for the ackerman

        quadSearch.graphSearch();

        // Total distance for both platforms
        double localMinDist = ackerSearch.getMinCost() + quadSearch.getMinCost();

        // Copy the lowest distance up to this permut
        if (localMinDist < minDist){
            minDist = localMinDist;

            reorder(ackerSearch.getBestPath(), localOrderAckerman);
            bestOrderAckerman = localOrderAckerman;

            reorder(quadSearch.getBestPath(), localOrderQuadcopter);
            bestOrderQuadcopter = localOrderQuadcopter;
        }
    } while (nextPermutation(assignment));

    // Update goals vector member variable of goal
    minCost_ = minDist;
    bestOrderAckerman_ = bestOrderAckerman;
    bestOrderQuadcopter_ = bestOrderQuadcopter;
    
    // Stop the timer
    auto end_time = std::chrono::high_resolution_clock::now();

    // Calculate the duration in seconds 
    timeElapsed_ = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();

    // Returns of whether found any valid permutation
    return foundAny;
}


std::vector<int> SuperSearch::getBestOrderAckerman(){
    return bestOrderAckerman_;
}

std::vector<int> SuperSearch::getBestOrderQuadcopter(){
    return bestOrderQuadcopter_;
}


// // // // // // // // //  Helper functions // // // // // // // // 

// Change the index of goals from the local permutation of goals to the index of goals_
bool SuperSearch::reorder(const std::vector<int>& local, std::vector<int>& global){
    
    if (local.size() != global.size()) return false;

    std::vector<int> temp(global.size());
    for (int i = 0; i < global.size(); i++)
        temp.at(i) = global.at(local.at(i));
    global = temp;

    return true;
}

// Print out the permutations
void SuperSearch::printVector(const std::vector<bool>& v) {
    for (const auto& elem : v) {
        std::cout << elem << ' ';
    }
    std::cout << std::endl;
}

// Proceed to the next permuations of the vector of bool
bool SuperSearch::nextPermutation(std::vector<bool>& v) {
    for (int i = v.size() - 1; i >= 0; --i) {
        if (!v.at(i)) {
            v.at(i) = true;
            return true;
        } else {
            v.at(i) = false;
        }
    }
    return false; // All permutations have been generated
}
// // // // // // // // // // // // // // // // // // // // // // // // 
