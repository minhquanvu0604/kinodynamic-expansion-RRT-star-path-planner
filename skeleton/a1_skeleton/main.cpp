#include "ackerman.h"
#include "mission.h"
#include <vector>
#include "pfms_types.h"
#include <iostream>

using std::vector;
using std::cout;
using std::endl;

int main(int argc, char *argv[]) {

    if(argc !=3){
        cout << " Not arguments given on command line." << endl;
        cout << " usage: " << argv[0] << "<x> <y>" << endl;
        return 0;
    }

    
    std::vector<ControllerInterface*> controllers;

    controllers.push_back(new Ackerman());

    if (controllers.at(0)->getPlatformType() == pfms::PlatformType::ACKERMAN){
        std::cout << "Created ACKERMAN" << std::endl;
    }
    else{
        std::cout << "What monster have we created!" << std::endl;
    }

    Mission mission(controllers);
    vector<pfms::geometry_msgs::Point*> points;

    pfms::geometry_msgs::Point point;
    point.x=atof(argv[1]);
    point.y=atof(argv[2]);

    points.push_back(&point);

    mission.setGoals(points);

    bool OK = mission.runMission();

    if(OK){
        std::cout << "Controller possibly reached goal" << std::endl;
    }
    else {
        std::cout << "Controller CAN NOT reach goal" << std::endl;
    }

    //The ackerman should be within 0.5m of goal position when completing motion.


    return 0;
}
