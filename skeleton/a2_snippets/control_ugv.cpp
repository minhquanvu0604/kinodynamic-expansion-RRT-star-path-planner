/*!
 *  Control UGV Example
 *  \details
 * This example code allows user to input a goal on command line
 * Audi is the controlled via constant steering and 0.1 throttle to reach within 0.1m of the goal
 * Upon reaching the distance to goal the Audi applies full brakes until velocity is near zero (0.01m/s)
 * To run ./command_ugv 0.0 10.0 (will drive Audi to goal x=0 y=10)
 * 
 * The code DOES NOT have any guarantees on the level of control (this is crude and may not reach
 * goals within a tolerance). The primary aim of the code is to show capability to compute correct steering.
 * 
 *  \author    Alen Alempijevic
 *  \version   1.01-0
 *  \date      2023-04-11
 *  \pre       none
 *  \bug       none 
 */

#include "pipes.h"
#include "audi.h"
#include <iostream>
#include <stdlib.h>
#include <thread>
#include <chrono>

using std::cout;
using std::endl;
using pfms::commands::UGV; // we state this here to be able to refer to UGV instead of the full name
using pfms::geometry_msgs::Point;

int main(int argc, char *argv[]) {
// ./control_ugv 10 20

    if(argc !=3){
        cout << " Not arguments given on command line." << endl;
        cout << " usage: " << argv[0] << " <goal_x> <goal_y>" << endl;
        return 0;
    }

    /* produce goal from command line parameters */
    Point goal {
        atof(argv[1]),
        atof(argv[2])
    };

    //! Created a pointer to a Pipe 
    Pipes* pipesPtr = new Pipes();
    // Platform type is Ackerman
    pfms::PlatformType type = pfms::PlatformType::ACKERMAN;
    // Create straucture for odometry data, needed to be passed to read command
    pfms::nav_msgs::Odometry odo;


    bool OK  =  pipesPtr->read(odo,type);

    if(!OK){ /// If we can't read odometry, terminating program
        cout << "Unable to read odometry, is the simulator running?";
        exit(1);
    }

    double steering; 
    double distance; 
    unsigned int seq=0;

    // We create an object audi of class Audi
    Audi audi;
    // We can call all public functions of audi, such as compute steering
    // you can find the description of this function in audi.h
    audi.computeSteering(odo,goal,steering,distance);

    cout << "Starting distance to goal:" << distance << " [m]" << endl;
    
    
    while(std::fabs(distance)>1.0){ // Will loop until distance within 1m of goal

        // Reading odometry
        bool OK  =  pipesPtr->read(odo,type);
        // small sleep just to not overwhelm the pipe
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        // Compute steering 
        OK = audi.computeSteering(odo,goal,steering,distance);

        if(!OK){
            cout << "WARNING: steering angle exceeds limits" << endl;
            cout << "Being capped to max steering angle" << endl;
        }

        //Prepare the UGV message
        UGV ugv {
                    seq++,  //increment the sequence number
                    0.0,    //no brake
                    steering,//steering
                    0.1      // throttle   
                 };

        // Send the command 
        pipesPtr->send(ugv);
        // small sleep just to not overwhelm the pipe
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        // only for analysis we can print distance to goal
        cout << "d:" << distance << endl;
        cout << "steering: " << steering << endl;
        
    }
    
    double v = std::pow(std::pow(odo.linear.x,2)+ std::pow(odo.linear.y,2),0.5);

    // Running another loop to stop the car (we can check the velocity to see if it is stopped)
    while (v>0.01){
        //Prepare the UGV message
        UGV ugv {
                    seq++,//increment the sequence number
                    8000,//maximum break
                    0,  //zero steering
                    0   //no throttle
                 };


        // Send the command 
        pipesPtr->send(ugv);
        // small sleep just to not overwhelm the pipe
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        // compute velocity
        v = std::pow(std::pow(odo.linear.x,2)+ std::pow(odo.linear.y,2),0.5);
        // only for analysis we can print velocity
        //cout << "v:" << v << endl;
    }

   return 0;
}
