// Includes std::cout and friends so we can output to console
#include <iostream>
#include "aircon.h"

// Every executable needs a main function which returns an int
int main (int argc,char** argv) {

    Aircon aircon;

    //! Obtain value of desired temperature
    std::cout << "What is the desired temperature? (1-50)" << std::endl;
    double desired_temp;
    std::cin >>  desired_temp;
    aircon.setDesiredTemp(desired_temp);
    std::cout << "Desired temperature entered:" << desired_temp << std::endl;
    std::cout << "Current temperature:" << aircon.getCurrentTemp() << std::endl;

    while(!aircon.run()){
        std::cout << aircon.getInfoString() << std::endl;
    }


    return 0;
}
