#include <iostream>
#include <vector>

#include "display.h"
#include "tf.h"
#include "tf2.h"
#include "types.h"
#include "analysis.h"

using std::vector;

void printInfo(RangeBearingStamped rb,Point bogie){

         std::cout << rb.timestamp << " [r,b]=[" << rb.range <<
                      "," << rb.bearing*180/M_PI << "]" << std::endl;

         std::cout << "pose [x,y]=[" << bogie.x << "," << bogie.y << "]" << std::endl;


}
int main (void) {


    Display display(1);

    Pose aircraft;
    aircraft.position = {0,0,0};
    aircraft.orientation = tf::yawToQuaternion(M_PI/4);

    std::vector<Point> bogies;
    { // WHAT IS THIS SCOPE USED FOR?
        std::vector<RangeBearingStamped> rbVec = display.scan(aircraft);
        for (auto rb : rbVec){
            Point bogie = tf2::local2Global(rb,aircraft);
            printInfo(rb,bogie);
            bogies.push_back(bogie);
        }
    }

    Analysis analysis(bogies);
    vector<double> times = analysis.timeToImpact(aircraft);

    AdjacencyList graph = analysis.exportGraph();

    int node =0;
    for (auto edges : graph){
        std::cout << node << std::endl;
        for(auto edge : edges){
            std::cout << edge.second << " " << edge.first << std::endl;
        }
    }

    return 0;
}
