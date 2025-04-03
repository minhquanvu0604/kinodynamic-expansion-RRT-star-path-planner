#ifndef ANALYSIS_H
#define ANALYSIS_H

#include <vector>
#include <thread>
#include "types.h"
#include "display.h"

using geometry_msgs::Pose;
using geometry_msgs::Point;
using std::pair;

typedef pair<double, unsigned int> EdgeInfo;
typedef vector<vector<EdgeInfo> > AdjacencyList;

class Analysis
{
public:
    /**
     * @brief Constructor for class
     * @param vector of Point(s) thaty are goals
     */
    Analysis(std::vector<Point> goals);

    /**
     * @brief Determines time to impact (time to reach each goal)
     * @note Please read README.md which has complete description of the task.
     * For each goal the computation for time to impact has to take into account the distance
     * and angle for each goal from the origin pose.
     *
     * @param origin for the search
     * @return time to impact for each goal(s) (this vector has same size as the poses supplied)
     * each element is time to impact for the respective goal
     */
    std::vector<double> timeToImpact(Pose origin);


    /**
     * @brief
     * Creates a graph of all goals, all nodes are interconnected
     * if number of nodes is N then each node is connected to N-1 nodes.
     * Store graph using the #AdjacencyList.
     *
     * In the graph:
     * 1) the outer vector contains an entry for all nodes // like our Week 5 BFS/DFS example
     * 2) the inner vector has edges a pair, where the pairs contains
     *  * a metric (we will use Euclidian distance between the nodes) - as fisrt element
     *  * the node id the edge is connecting to - as second element
     * @return An adjacencylist interconecting all nodes
     */
    AdjacencyList exportGraph();


private:
    std::vector<Point> goals_;

    double euclideanDistance(Point p1, Point p2);
};

#endif // ANALYSIS_H
