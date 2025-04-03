#ifndef ANALYSIS_H
#define ANALYSIS_H

#include <vector>
#include <thread>
#include "dep/include/car.h"
#include "display_race.h"

class Analysis
{
public:
    /**
     * @brief Constructor for class
     * @param vector of pointers to CarInterface
     */
    Analysis(std::vector<CarInterface *> cars);
    /**
     * @brief Constructor for class
     * @param vector of pointers to CarInterface
     * @param raceDisplay pointer, enabling us to visualise the race
     */
    Analysis(std::vector<CarInterface*> cars,std::shared_ptr<DisplayRace> raceDisplay);

    /**
     * @brief Standard drag race
     * Create a drag racing algorithm, where each vehicle will race a distance of 100m. So the logic for `dragRace` is:
     * 1. Keep accelerating each car
     * 2. Record the order of the car ID's as they cross the finish line at 100m
     * 3. When each of the cars covers 100m the race is finished, we declare the winner and terminate the race.
     * Return the order of vehicle IDs 
     * @param set distance for race
     * @return order of cars (the vector has same size as the number of cars in race), each element is the position of the respective car in the race
     */
    std::vector<unsigned int> dragRace(double distance);

    /**
     * @brief Stop all the cars (this function ensure their speed is zero)
     */
    void stopAllCars();

    /**
     * @brief
     * Create a drag race algortithm, where all cars get to top speed and then back to zero, quickest to do so wins race. 
     * So the logic for the `zeroTopZeroRace` is :
     * 1. Accelerate all vehicle
     * 2. As each individual vehicle reaches top speed, decelerate that vehicle.
     * 3. When all vehicles have stopped, terminate the race
     * @return order of cars (the vector has same size as the number of cars in race), each element is the position of the respective car in the race
     */
    std::vector<unsigned int> zeroTopZeroRace();


    /**
     * @brief
     * Sort the cars by their odometry
     * @return asceding order of cars (the vector has same size as the number of cars in race), each element in vector contains the car ID, positions in vector is in order using odometry (lowest position has lowest number of kilometers)
     */
    std::vector<unsigned int> sortByOdometry();

    /**
     * @brief
     * Demo of racing, accelerates the cars 300 times, deccelerates 600 times.
     */
    void demoRace();

private:
    std::vector<CarInterface*> cars_;
    std::shared_ptr<DisplayRace> raceDisplay_;
};

#endif // ANALYSIS_H
