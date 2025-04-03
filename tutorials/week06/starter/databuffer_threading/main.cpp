#include <iostream>
#include <thread>
#include <chrono>
#include <random>

#include "databuffer.h"

using std::vector;

void printValues(vector<double> values) {
    std::cout << "[" << values.size() << "] ";
    for(auto val:values){
        std::cout << val<< " ";
    }
    std::cout << std::endl;
}

int main ()
{
    // Create the shared buffer which contains its own mutex
    DataBuffer dataBuffer(10,70,200);

    std::default_random_engine generator;
    std::uniform_real_distribution<double> distribution(0,100);

    while (true) {

        //We generate 100 numbers at a time
        std::vector<double> values;
        values.reserve(100);
        for(unsigned i=0;i<100;i++){
            values.push_back(distribution(generator));
        }
        // //std::cout << "Pushing vector" << std::endl;
        dataBuffer.addValues(values);
        //std::cout << "Print vector" << std::endl;
       printValues(dataBuffer.getValues());
        std::this_thread::sleep_for (std::chrono::milliseconds(500));

    }


    return 0;
}



