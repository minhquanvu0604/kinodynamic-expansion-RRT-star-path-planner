
#include "pipes.h"
#include <iostream>
#include <stdlib.h>
#include <thread>
#include <chrono>
#include "linkcommand.h"
#include "test_helper.h"
#include "pfms_types.h"

using std::cout;
using std::endl;
using pfms::geometry_msgs::Goal;


int main(int argc, char *argv[]) {

    // The first odometry is -1.14887e-05, -0.000448213, 0.000278163

    LinkCommand* linkCommand = new LinkCommand(true);
    {
        Odometry odo = populateOdoUGV(-0.1, -0.000448213, 0);
        linkCommand->writeCommand(odo);
    }

    std::cout << "Populate to initial position completed!" << std::endl;
}