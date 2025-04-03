#include <cmath>
#include <iostream>
#include <utility>

#include "mission.h"
#include "pipes.h"
#include "ackermansearch.h"
#include "quadcoptersearch.h"
#include "supersearch.h"

// In SuperSearch, initialize two 
//    * objects of Ackerman and Quadcopter search and implement searching for all the distribution (permutations of the goals)


/**
 * @brief Constructor
 * 
 * Receive controller pointer as an argument and use it to initialize the corresponding member variable
 * Also set default value for the mission objective as BASIC    
 *  
 * @param controllers A vector of pointer to ControllerInterface object 
 */
Mission::Mission(std::vector<ControllerInterface*> controllers) : controllers_{controllers}, 
                                                                missionObjective_{mission::BASIC}{}


/**
   * @brief Accepts the container of goals and the corresponding platform
   *
   * Do additional goal allocation according the objective of the mission, since different objectives requires
   * different ways to store and distribut goals
   * 
   * Conduct graph search accordingly if the objective is either ADVANCED or SUPER. Do so by initializing objects of 
   * the corresponding graph search (AckermanSearch, QuadcopterSearch and SuperSearch). For all the searches, the minimum 
   * cost and the best order for each platform to visit is passed to member variable of Mission class 
   * 
   * Calculate the total distance that the platforms have to travel given the goal vector processed in the above part
   * 
   * @param goals Vector of Points 
   * @param platform Platform type
 */
void Mission::setGoals(std::vector<pfms::geometry_msgs::Point> goals, pfms::PlatformType platform){

    // // // // // // // // // // SETTING GOALS // // // // // // // // // // // // // // // // // // // // // // // // // // // //

    // Set the goals for the BASIC, ADVANCED & SUPER MODE
    switch (platform){
        case pfms::ACKERMAN:
            goalsAckerman_ = goals;
            break;

        case pfms::QUADCOPTER:
            goalsQuadcopter_ = goals;
            break;
        default:
            break;
    }
    // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // //



    // // // // // // // // // // ALGORITHM // // // // // // // // // // // // // // // // // // // // // // // // // // // //

    if (missionObjective_ == mission::ADVANCED){

        // Conducting graph search, needs to find the corresponding graph search for different platforms
        if (platform == pfms::ACKERMAN){

            // Create a GraphSearch object
            AckermanSearch ackerSearch;

            // Set goals for the graph search
            ackerSearch.setGoals(goals);
            
            // Find the Ackerman pointer in controllers_ vector            
            for (auto controller : controllers_){
                if (controller->getPlatformType() == pfms::ACKERMAN){
                    ackerSearch.setPtr(controller);
                    break;
                    }
                }

            // Implement graph search
            ackerSearch.graphSearch();

            // Get the best path after graph search
            bestOrderAckerman_ = ackerSearch.getBestPath();

            // Change the order in goalsAckerman_ according to the new goal order provided by the graph search
            ackerSearch.reorderPoint(bestOrderAckerman_, goalsAckerman_);
                        
        }
        else if (platform == pfms::QUADCOPTER){

            // Create a GraphSearch object
            QuadcopterSearch quadSearch;

            // Set goals for the graph search
            quadSearch.setGoals(goals);

            // Find the Quadcopter pointer in controllers_ vector            
            for (auto controller : controllers_){
                if (controller->getPlatformType() == pfms::QUADCOPTER){
                    quadSearch.setPtr(controller);
                    break;
                    }
                }

            // Implement graph search
            quadSearch.graphSearch();

            // Get the best path after graph search
            bestOrderQuadcopter_ = quadSearch.getBestPath();

            // Change the order in goalsAckerman_ according to the new goal order provided by the graph search
            quadSearch.reorderPoint(bestOrderQuadcopter_, goalsQuadcopter_);
        }
    }


    if (missionObjective_ == mission::SUPER){

        // Only proceeds if there is two controllers
        if (controllers_.size() != 2) {
            return;
        }

        SuperSearch search;

        // Find the platform that each pointer points to and set pointer member variable for search object
        int ackermanPosition = -1;
        for (int i = 0; i < controllers_.size(); i++){
                if (controllers_.at(i)->getPlatformType() == pfms::ACKERMAN){
                    ackermanPosition = i;
                    break;
                    }
                }
        if (!ackermanPosition) search.setPtrs(controllers_.at(0), controllers_.at(1));
        else search.setPtrs(controllers_.at(0), controllers_.at(1));

        // Set goals
        search.setGoals(goalsAckerman_);
        search.setGoals(goalsQuadcopter_);

        // One set of unified goals 
        std::vector<pfms::geometry_msgs::Point> goalsSuper;
        goalsSuper.insert(goalsSuper.begin(), goalsAckerman_.begin(), goalsAckerman_.end());
        goalsSuper.insert(goalsSuper.begin(), goalsQuadcopter_.begin(), goalsQuadcopter_.end());


        // Implement graph search
        search.graphSearch();

        // Get the min cost of the graph search
        minCost_ = search.getMinCost();

        // Update member variable which will be used for getPlatformAssociation()
        bestOrderAckerman_ = search.getBestOrderAckerman();
        bestOrderQuadcopter_ = search.getBestOrderQuadcopter();

        // Change the order in goalsAckerman_ and goalsQuadcopter_ according to the new goal order provided by the graph search
        goalsAckerman_ = search.reorderPoint(bestOrderAckerman_, goalsSuper);
        goalsQuadcopter_ = search.reorderPoint(bestOrderQuadcopter_, goalsSuper);
    }

    // Calculate total distance for all the modes
    // Calling controller's set goals here
    switch (platform){

        case pfms::ACKERMAN:

            for (auto controller : controllers_){
                if (controller->getPlatformType() == pfms::ACKERMAN){
                    // Set goals
                    controller->setGoals(goalsAckerman_);

                    auto odo = controller->getOdometry();
                    double distance = 0;
                    double time = 0;

                    for (int g = 0; g < goalsAckerman_.size(); g++){

                        // Check whether points are reachable
                        bool OK = controller->checkOriginToDestination(odo, goals.at(g), distance, time, odo);
                        if (!OK) reachable_= false;

                        // 
                        totalDistanceAckerman_ += distance;
                    }
                }
            }
            break;
        

        case pfms::QUADCOPTER:    

            for (auto controller : controllers_){
                if (controller->getPlatformType() == pfms::QUADCOPTER){
                    // Set goals
                    controller->setGoals(goalsQuadcopter_);

                    // Calculate total distance
                    auto odo = controller->getOdometry();
                    totalDistanceQuadcopter_ = std::sqrt(std::pow(odo.position.x - goalsQuadcopter_.at(0).x, 2) 
                                        + std::pow(odo.position.y - goalsQuadcopter_.at(0).y, 2));;

                    for (int g = 0; g < goalsQuadcopter_.size()-1; g++){
                        double distance = std::sqrt(std::pow(goalsQuadcopter_.at(g).x - goalsQuadcopter_.at(g+1).x, 2) 
                                        + std::pow(goalsQuadcopter_.at(g).y - goalsQuadcopter_.at(g+1).y, 2));
                        totalDistanceQuadcopter_ += distance;
                    }
                }
            }
            break;

        default:
            break;
    }
}


/**
 * @brief Runs the mission, non blocking call
 * 
 * The function checks the reachable_ member variable. If the variable is false, which indicates that the goal list is 
 * not achievable, the function returns false. Otherwise, it calls the run function of all the controllers via the vector
 * of ControllerInterface pointer
 * 
 * As this function serves to trigger the running of platforms, it does not block the main thread 
 * 
 * @return bool indicating mission can be completed (false if mission not possible)
*/
bool Mission::run(){
    if (!reachable_) {
        return false;
    }
    
    for (auto controller : controllers_)
        controller->run();
    
    return true;
}


/**
 * @brief Status of the mission
 * 
 * Returns mission completion status (indicating percentage of completion of task) by each platform @sa setGoals
 * The percentage is heuristically offset to reach 100 when the platform has completed the goals and turns itself back to idle
 * 
 * @return Vector with each element of vector corresponding to a platform. The value is percent of completed distance of entire mission for the 
 * corresponding platform value between 0-100.
 */
std::vector<unsigned int> Mission::status(void){
    std::vector<unsigned int> stat;

    for (auto controller : controllers_){

        if (controller->getPlatformType() == pfms::ACKERMAN){
            unsigned int percent = static_cast<unsigned int>(controller->distanceTravelled() * 100 / (totalDistanceAckerman_ - 0.5));            
        
            // Result treatment
            if (percent >= 95 && controller->status() == pfms::IDLE) percent = 100;

            stat.push_back(percent);
        }

        else if (controller->getPlatformType() == pfms::QUADCOPTER){
            unsigned int percent = static_cast<unsigned int>(controller->distanceTravelled() * 100 / totalDistanceQuadcopter_);
            
            // Result treatment
            if (percent >= 95 && controller->status() == pfms::IDLE) percent = 100;

            stat.push_back(percent);
        }
    }
    return stat;
}


/**
 * @brief Set mission objective
 * 
 * Also clears the goal vector of Ackerman and Quadcopter
 */
void Mission::setMissionObjective(mission::Objective objective){
    missionObjective_ = objective;

    goalsAckerman_.clear();
    goalsQuadcopter_.clear();
}


/**
 * @brief Returns a vector of same size as number of controllers (platforms).
 * The values in the vector correspond to the total distance travelled by the corresponding platform
 * from the time of starting the program.
 *
 * The function implement the for loop to indicate which platforms do the pointers in controllers_ belong to 
 * and pushing back to store the distance travelled in order with the platform order
 * 
 * @return std::vector<double> - each element is distance travelled for each platform [m]
 */
std::vector<double> Mission::getDistanceTravelled(){
    std::vector<double> vect;
    for (auto controller : controllers_)
        vect.push_back(controller->distanceTravelled());
    return vect;
}


/**
 * @brief Returns a vector of same size as number of controllers (platforms).
 * The values in the vector correspond to the time the corresponding platfore has been moving
 * from the time the program started. Moving means the platform was not stationary.
 *
 * The function implement the for loop to indicate which platforms do the pointers in controllers_ belong to 
 * and pushing back to store the time travelled in order with the platform order
 * 
 * @return std::vector<double> - each elemtn distance travelled for each vehicle [m]
 */
std::vector<double> Mission::getTimeMoving(){
    std::vector<double> vect;
    for (auto controller : controllers_)
        vect.push_back(controller->timeTravelled());
    return vect;
}


/**
 * @brief Returns a vector of same size as the total number of goals. The values in the vector
 * are a pair, corresponding to the controller that is completing the goal and the goal number
 * (the goal number is derived from the order of all goals supplied)
 *
 * This function is used as an interface to display the goal order after the sorting occurs in ADVANCED and SUPER MODE
 * For BASIC MODE, the function simply returns the goal order similar to the order initially set in @sa setGoals, but in the 
 * format restricted by this function
 * 
 * The function utilizes the vector of integers member variable set by the graph search
 * 
 * @return vector of pair of int as per brief
 */
std::vector<std::pair<int, int>> Mission::getPlatformGoalAssociation(){

    unsigned int vecSize = goalsAckerman_.size() + goalsQuadcopter_.size();
    std::vector<std::pair<int, int>> association(vecSize, {0, 0});

    if (missionObjective_ == mission::BASIC){
        for (int i = 0; i < vecSize; i++)
        association.at(i).second = i;
    }
    else {
        // Set the second element of the pair that indicates which platforms is referred to
        // Ackerman is 0, Quadcopter is 1
        for (int i = goalsAckerman_.size(); i < vecSize; i++)
            association.at(i).first = 1;
            
        for (int i = 0; i < bestOrderAckerman_.size(); i++)
            association.at(i).second = bestOrderAckerman_.at(i);

        for (int i = 0; i < bestOrderQuadcopter_.size(); i++)
            association.at(i + bestOrderAckerman_.size()).second = bestOrderQuadcopter_.at(i);
    }

    return association;
}