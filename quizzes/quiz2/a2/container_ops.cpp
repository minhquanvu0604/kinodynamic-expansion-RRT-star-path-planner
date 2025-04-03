#include "container_ops.h"
#include <random>   // Includes the random number generator
#include <chrono>   // Includes the system clock
#include <iterator> // For using iterators in Lambda functions (ADVANCED)
#include <algorithm> //Can use algorithms on STL containers


void populateContainer(std::deque<double>& container, unsigned int num_values, double element){
    container.insert(container.begin(), num_values, element);
}

void bubbleSortContainer(std::deque<double>& container){
    bool swapped;

    for (int step = 0; step < container.size() - 1; step++){
        swapped = false;
        for (int i = 0; i < container.size() - step - 1; i++){
            if (container.at(i) > container.at(i + 1)){
                double temp = container.at(i);
                container.at(i) = container.at(i + 1);
                container.at(i + 1) = temp;

                swapped = true;
            } 
        }
        
        if (swapped == 0) break;
    }
}


