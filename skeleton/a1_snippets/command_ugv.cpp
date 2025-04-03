// Helper utility to send comamnd to ugv
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This executable shows the use of the pipes library to send commands
// and receive odometry from the ugv platform




///// NOTE /////

//roslaunch gazebo_tf multi.launch
// After finishing all the commands, the steering goes back to 0

#include "pipes.h"
#include <iostream>
#include <stdlib.h>
#include <thread>
#include <chrono>

using std::cout;
using std::endl;
using pfms::commands::UGV; // we state this here to be able to refer to UGV instead of the full name

int main(int argc, char *argv[]) { 
    // TEST RUN ./command_ugv 1000 0 0.5 0.1
    // ./command_ugv 500 0 0.5 0.8
    // steering: from -1.6 to 1.6 (LOCK_TO_LOCK_REV)

    if(argc !=5){
        cout << " Not arguments given on command line." << endl;
        cout << " usage: " << argv[0] << "<repeats> <brake> <steering> <throttle>" << endl; 
        return 0;
    }


    //! Created a pointer to a Pipe 
    Pipes* pipesPtr = new Pipes(); // WHY IS IT DYNAMICALLY ALLOCATED
    pfms::nav_msgs::Odometry odo;
    pfms::PlatformType type = pfms::PlatformType::ACKERMAN;

    unsigned long i = 0;
    // /* produce messages */
    for(i = 0; i < atoi(argv[1]); i ++) { // SOMETIMES MOVE OUT OF THE TRAJECTORY
        UGV ugv {
                    i,
                    atof(argv[2]),
                    atof(argv[3]),
                    atof(argv[4])
                };
                // Steer angle negative -> turn right 
                // Steer angle positive -> turn left  
        pipesPtr->send(ugv); 
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        cout << "wrote:" << i << endl;

        // std::cout << "new" << std::endl; // For debugging

        bool OK  =  pipesPtr->read(odo,type); // Can still read without send(ugv)

        if(OK){
            std::cout << "i seq x,y,yaw,vx,vy: " <<
                i << " " <<
                odo.seq << " " <<
                odo.position.x << " " << // positive upward
                odo.position.y << " " << // positive leftward
                odo.yaw << " " << // from -pi to pi, positive counterclockwise
                odo.linear.x << " " <<
                odo.linear.y << std::endl;
        }
        std::this_thread::sleep_for (std::chrono::milliseconds(10));        
    }
    
   return 0;
}
