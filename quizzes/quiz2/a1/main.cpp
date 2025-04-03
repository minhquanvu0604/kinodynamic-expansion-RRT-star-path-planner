

#include <iostream>
#include <thread>         // std::this_thread::sleep_for
#include <vector>

#include "analysis.h"
#include "car.h"

void printOrderStats(std::vector<CarInterface*> cars,std::vector<unsigned int> order){
    for (unsigned int i=0;i<cars.size();i++){
         std::cout << cars.at(i)->getMake() << " " << cars.at(i)->getModel() <<
                      " odo:" << cars.at(i)->getOdometry() <<
                      " speed:" << cars.at(i)->getCurrentSpeed() <<
                      " position:" << order.at(i) << std::endl;
    }
}

int main (void) {


    std::vector<CarInterface*> cars;

    //! @todo
    //! TASK 1
    //! Create 3 cars with follwing specifications

    // Mercedes - C180
    // ID : 20
    // height = 1.45 m, width = 1.77 m, power = 143 HP, drag coefficient = 0.29, weight = 1200 kg

    // Bugatti - Veyron
    // ID : 25
    // height = 1.19 m, width = 2.00 m, power P = 1200 HP, drag coefficient = 0.35, weight = 2200 kg

    // Toyota - Yaris_WRsC
    // ID : 2
    // height = 1.19 m, width = 1.87 m, power P = 420 HP, drag coefficient = 0.30, weight = 1190 kg
    cars.push_back(new Car("merc", "c180", 20, 1.45,1.77,143,0.29,1200));
    cars.push_back(new Car("bugatti", "veyron", 25, 1.19,2.00,1200,0.35,2200));
    cars.push_back(new Car("toyota", "yaris", 2, 1.19,1.87,420,0.30,1190));

    // std::cout << cars.at(0)->getMake() << std::endl;

    

    std::shared_ptr<DisplayRace> raceDisplay(new DisplayRace(cars));

//    //We create a pointer to the Radar, will use a shared pointer here
    std::shared_ptr<Analysis> analysisPtr(new Analysis(cars,raceDisplay));

//    // The below is just a demo race, to show how to accelerate and decelerate the cars.
//    // You have to keep accelerating to go fast, and decelerate to slow down
//    // STUDENTS: Comment it out, not needed for other tasks.
    // analysisPtr->demoRace();

//    // We call TASK 1
//     {
//        std::vector<unsigned int> order = analysisPtr->sortByOdometry();
//        printOrderStats(cars,order);
//     }    

//    // We call TASK 2
//    {
//        std::vector<unsigned int> order = analysisPtr->dragRace(100.0);
//        printOrderStats(cars,order);
//    }

//    // We call TASK 3
//    analysisPtr->stopAllCars();

//    // We call TASK 4
//    {
//        std::vector<unsigned int> order = analysisPtr->zeroTopZeroRace();
//        printOrderStats(cars,order);
//    }

    // // TEST
    // std::cout << "SPEED: " << (cars.at(0))->getCurrentSpeed() << std::endl;
    // std::cout << "SPEED: " << (cars.at(1))->getCurrentSpeed() << std::endl;
    // std::cout << "SPEED: " << (cars.at(2))->getCurrentSpeed() << std::endl;



    return 0;
}
