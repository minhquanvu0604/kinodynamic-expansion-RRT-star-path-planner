#include <limits>
#include "analysis.h"

using std::vector;
using std::pair;

//! @todo
//! TASK 1 - We need to store cars in the Analysis class, how to do this?
Analysis::Analysis(std::vector<CarInterface*> cars) :
    cars_(cars),raceDisplay_(nullptr)
{

}

Analysis::Analysis(std::vector<CarInterface*> cars,std::shared_ptr<DisplayRace> raceDisplay) :
    cars_(cars),raceDisplay_(raceDisplay)
{

}

//! @todo
//! TASK 1 - Refer to README.md and the Header file for full description
std::vector<unsigned int> Analysis::sortByOdometry(){

    vector<pair<double, int> > vp; // Creating vector of pairs
    for (int i = 0; i < cars_.size(); i++)
        vp.push_back(std::make_pair((cars_.at(i))->getOdometry(), i)); //The value to be sorted as first element, and the previous index as second element

    sort(vp.begin(), vp.end());

    //  //Printing vp 
    // std::cout << "Element\t"
    //      << "index" << std::endl;
    // for (int i = 0; i < vp.size(); i++) {
    //     std::cout << vp[i].first << "\t"
    //          << vp[i].second << std::endl;
    // }

    std::vector<unsigned int> order(cars_.size(),0); //Creating a vector, same size as cars with all zeros

    for (int i = 0; i < cars_.size(); i++)
        order.at(i) = (cars_.at((vp.at(i)).second))->getID();
    
    // //Printing order
    // for (int i = 0; i < cars_.size(); i++)
    //     std::cout << order.at(i) << std::endl;

    return order;


}

//! @todo
//! TASK 2 - Refer to README.md and the Header file for full description
std::vector<unsigned int> Analysis::dragRace(double distance){

    //Creating a vector, same size as cars with all zeros
    std::vector<unsigned int> order(cars_.size(),0);

    //Vector storing initial odometry of each car
    std::vector<double> initialOdometry(cars_.size(),0);
    for (int i = 0; i < cars_.size(); i++){
            initialOdometry.at(i) = (cars_.at(i))->getOdometry();
    }

    unsigned int numCarFinished = 0;

    //Flag indicating whether each car has finished travelling the distance
    std::vector<bool> finished(cars_.size(), false);

    while (true){
        for (int i = 0; i < cars_.size(); i++){
            (cars_.at(i))->accelerate();
            if (!(finished.at(i)) // LIGHT COMPUTATION SO EVALUATED FIRST
                    && (cars_.at(i))->getOdometry() - initialOdometry.at(i) >= distance){ // HEAVY COMPUTATION SO EVALUATED SECOND
                order.at(numCarFinished) = (cars_.at(i))->getID();
                numCarFinished++;
                finished.at(i) = true;
            }
        }
        if (numCarFinished == cars_.size()) break;
    }
    return order;
}


//! @todo
//! TASK 3 - Refer to README.md and the Header file for full description
void Analysis::stopAllCars(){
    while (true)
    {
        unsigned int numCarStopped = 0;

        for (int i = 0; i < cars_.size(); i++){
            if ((cars_.at(i))->getCurrentSpeed() > std::numeric_limits<double>::epsilon())
                (cars_.at(i))->decelerate();
            else numCarStopped++;            
        }

        if (numCarStopped == cars_.size()) break;
    }
    
}

//! @todo
//! TASK 4 - Refer to README.md and the Header file for full description
std::vector<unsigned int> Analysis::zeroTopZeroRace(){

    //Creating a vector, same size as cars with all zeros
    std::vector<unsigned int> order(cars_.size(),0); // Creating a vector, same size as cars with all zeros

    std::vector<bool> reachedTopSpeed(cars_.size(), false); // Reached top speed?

    unsigned int finishedRace = 0; // Number of cars that have finished the race

    while (true){
        for (int i = 0; i < cars_.size(); i++){
            // One-time toggle if reached top speed
            if (!(reachedTopSpeed.at(i)) // Haven't reached top speed
                    && std::abs((cars_.at(i)->getTopSpeed()) - (cars_.at(i)->getCurrentSpeed())) > std::numeric_limits<double>::epsilon()){ // Current speed = top speed
                reachedTopSpeed.at(i) = true;
            }   

            // Accelerate or Decelerate
            if (!reachedTopSpeed.at(i)) (cars_.at(i))->accelerate();
            else {
                (cars_.at(i))->decelerate();

                // Whether finished the race, put into order 
                if ((cars_.at(i)->getCurrentSpeed()) <= std::numeric_limits<double>::epsilon()){// Current speed = 0
                    order.at(finishedRace) = (cars_.at(i))->getID(); 
                    finishedRace++;
                }
            }
        }
        if (finishedRace == cars_.size()) break;
    }

    return order;
}

// Demo code
void Analysis::demoRace(){


    //This is an example of how to draw 3 cars moving
    // accelerate 300 times
    unsigned int count=0;

    while(count < 300){
        for(auto car : cars_){
          car->accelerate();
        }
        if(raceDisplay_!=nullptr){
            raceDisplay_->updateDisplay();
        }
        count++;
    }

    // decelerate 600 times
    count =0;
    while(count < 600){
        for(auto car : cars_){
          car->decelerate();
        }
        if(raceDisplay_!=nullptr){
            raceDisplay_->updateDisplay();
        }
        count++;
    }

}
