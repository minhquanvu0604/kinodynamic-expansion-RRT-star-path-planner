#include <iostream>
#include <cmath>
#include <vector>
#include <chrono>

#include "mission.h"
#include "pipes.h"
#include "ackerman.h"
#include "pfms_types.h"


Mission::Mission(std::vector<ControllerInterface*> controllers) : controllers_{controllers}{}

void Mission::setGoals(std::vector<pfms::geometry_msgs::Point*> goals){
    goals_ = goals;
}

bool Mission::runMission(){
    for (int i = 0; i < controllers_.size(); i++){
        for (int j = 0; j < goals_.size(); j++){
            // // DEBUGGING    
            // for (int k = 0; k < 10; k++)
            //     std::cout << "STARTING GOAL " << k << std::endl;

            // Setting goals     
            bool goalOK = controllers_.at(i)->setGoal(*goals_.at(j));
            if (!goalOK) return false;

            // Reaching goals
            bool goalReached = controllers_.at(i)->reachGoal();
            if (!goalReached) return false;
                // for (int k = 0; k < 10; k++)
                //     std::cout << "GOAL " << k << "REACHED !!!";
        }
    }
    return true;
}

void Mission::setMissionObjective(mission::Objective objective){
    missionObjective_ = objective;
  }

std::vector<double> Mission::getDistanceTravelled(){
    std::vector<double> distanceTravelledVector(controllers_.size());

    for (int i = 0; i < controllers_.size(); i++)
        distanceTravelledVector.push_back((controllers_.at(i))->distanceTravelled());
    
    return distanceTravelledVector;
}

std::vector<double> Mission::getTimeMoving(){
    std::vector<double> timeTravelledVector(controllers_.size());

    for (int i = 0; i < controllers_.size(); i++)
        timeTravelledVector.push_back((controllers_.at(i))->timeInMotion());
    
    return timeTravelledVector;
}

std::vector<unsigned int> Mission::getPlatformGoalAssociation(){
    goalAssignment_ = std::vector<unsigned int>(goals_.size(), 0);
    return goalAssignment_;
}