#include "analysis.h"
#include "tf.h"
#include "tf2.h"

using std::vector;
using std::pair;
using geometry_msgs::Point;

Analysis::Analysis(std::vector<Point> goals) :
    goals_(goals)
{

}


//! @todo
//! TASK 3 - Refer to README.md and the Header file for full description
vector<double> Analysis::timeToImpact(Pose origin){

    //The consts you will need are
    //Display::OMEGA_MAX
    //Display::V_MAX

    vector<double> times;

    for (auto goal : goals_){
        auto rbs = tf2::global2local(goal,origin);
        double time = std::abs(rbs.bearing) * Display::OMEGA_MAX + rbs.range * Display::V_MAX;
        times.push_back(time);
    }

    return times;
}

//! @todo
//! TASK 4 - Refer to README.md and the Header file for full description
AdjacencyList Analysis::exportGraph(){

    //std::make_pair(odom.at(i), i));
    AdjacencyList graph;

    // Go through all the points
    for (int point = 0; point < goals_.size(); point++){
        vector<EdgeInfo> poi;

        // Go through all the edge infos
        for (int edge = 0; edge < goals_.size(); edge++){
            if (edge == point) continue;

            EdgeInfo ed;
            ed.first = euclideanDistance(goals_.at(point), goals_.at(edge));
            ed.second = edge;

            // std::cout << "The distance between point " << point << " and point " << edge << " is " <<
            //     ed.first << std::endl; 

            poi.push_back(ed);
        }

        graph.push_back(poi);
        
    }

    // std::cout << "graph.at(0).at(1).first = " << graph.at(0).at(1).first << std::endl;

    return graph;
}

double Analysis::euclideanDistance(Point p1, Point p2) {
    double dx = p1.x - p2.x;
    double dy = p1.y - p2.y;
    return sqrt(dx*dx + dy*dy);
}