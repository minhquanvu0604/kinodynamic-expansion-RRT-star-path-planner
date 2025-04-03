#include <iostream>
#include "data.h"
#include "sensor.h"
#include "analysis.h"

using namespace std;

int main ()
{
    pfms::Data data;// We create a data structure

    Sensor sensor(&data); // Passing the data structure to Sensor

    Analysis analysis(&data); // Passing the data structure to Analysis

    while(analysis.isRunning()){ // This checks if analysis is still running and only terminates when finished
      std::this_thread::sleep_for (std::chrono::milliseconds(500));
    }


    return 0;
}



