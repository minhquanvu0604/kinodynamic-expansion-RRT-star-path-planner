// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// producer example that sends data to UGV

#include "pipes.h"
#include <iostream>
#include <stdlib.h>
#include <thread>
#include <chrono>

using std::cout;
using std::endl;
using pfms::geometry_msgs::Goal;

int main(int argc, char *argv[]) {

    if(argc !=4){
        cout << " Not arguments given on command line." << endl;
        cout << " usage: " << argv[0] << "<seq_num> <goal_1_x> <goal_1_y>" << endl;
        return 0;
    }

    //! Created a pointer to data processing
    //std::shared_ptr<Pipes> pipesPtr(new Pipes();
    Pipes* pipesPtr = new Pipes();

    /* Lets send a goal to be shown on screen */
    Goal goal {static_cast<unsigned long>(atoi(argv[1])),
        {atof(argv[2]),atof(argv[3])}
         };
    pipesPtr->send(goal);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    
   return 0;
}
