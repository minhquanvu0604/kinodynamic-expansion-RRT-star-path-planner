#include "pipes.h"
#include <iostream>
#include <stdlib.h>
#include <thread>
#include <chrono>

using std::cout;
using std::endl;
using pfms::commands::UAV;

int main(int argc, char *argv[]) {

    if(argc !=6){
        cout << " Not arguments given on command line." << endl;
        cout << " usage: " << argv[0] << " <repeats> <turn_l_r> <move_l_r> <move_u_d> <move_f_b>" << endl;
        return 0;
    }

    std::shared_ptr<Pipes> pipesPtr = std::make_shared<Pipes>();
    pfms::nav_msgs::Odometry odo;
    pfms::PlatformType type = pfms::PlatformType::QUADCOPTER;

    //Let's take off here, we send the status to the platform
    // pfms::PlatformStatus status = pfms::PlatformStatus::TAKEOFF;
    // pipesPtr->send(status);

    unsigned long i = 0;

    /* We loop sending same message for the number of times requested */
    for(i = 0; i < atoi(argv[1]); i ++) {
        // We take the arguments supplied on command line and place them in the uav command message
        UAV uav {i,atof(argv[2]),atof(argv[3]),atof(argv[4]),atof(argv[5])};
        // Sending the commands
        pipesPtr->send(uav);
        // We wait for a short time, just to enable remaining of system to respond, recommended to wait
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        bool OK =  pipesPtr->read(odo,type);
        // We check if the odometry returned indicates the system is running
        if(!OK){
            break;
        }
        std::cout << "i seq x,y,yaw,vx,vy: " <<
            i << " " <<
            odo.seq << " " <<
            odo.position.x << " " <<
            odo.position.y << " " <<
            odo.yaw << " " <<
            odo.linear.x << " " <<
            odo.linear.y << " " <<
            odo.linear.z << std::endl;
        std::this_thread::sleep_for (std::chrono::milliseconds(10));
    }

    // At the end of moving the uav it is recommended to at least stop, and possibly land
    // CONSIDER: 
    //      How would hover? What command do you need to send and do you need to check odometry
    //      Would you land before hovering? What if you needed to land on a target?

    return 0;
}
