#include <iostream>
#include <thread>
#include <vector>
#include <iostream>

#include "radar.h"
#include "analysis.h"




int main (void){

    //We create a pointer to the Radar, will use a shared pointer here
    std::shared_ptr<Radar> radarPtr(new Radar());

    //Let's start the radar
    radarPtr->setMaxDistance(80.0);
    radarPtr->start();


    //! Created a pointer to Analsyis (as we need to pass this pointer as 2nd parameter to thread
    std::shared_ptr<Analysis> analysisPtr(new Analysis(radarPtr));

    double scanningSpeed=0;
    unsigned int samples=100;

    //! Create thread
    std::thread timing_thread(&Analysis::computeScanningSpeed,analysisPtr,samples,std::ref(scanningSpeed));

    timing_thread.join();

    std::cout << "queried [" << samples << "] samples from sesnor" << std::endl;
    std::cout << "refresh rate [" << scanningSpeed<< "] seconds" << std::endl;

    return 0;
}

