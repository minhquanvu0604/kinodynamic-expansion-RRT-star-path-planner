#include <iostream>
#include <vector>
#include <stack>
#include <set>
#include <queue>
#include <map>
#include <algorithm>

using std::vector;
using std::pair;
using std::stack;
using std::cout;
using std::endl;
using std::map;
using std::set;
using std::queue;

typedef vector<vector<pair<int,double>> > AdjacencyList;

//Printing the graph
void print_adjacency_list(AdjacencyList graph)
{
    // If stack is empty then return
    if (graph.empty())
        return;

    unsigned int nodeNum=0;
    for(auto edges : graph){
        std::cout << "[" << nodeNum++ << "] ";
        for(auto nodes : edges){
            std::cout << nodes.first << ":" << nodes.second << " ";
        }
        std::cout << std::endl ;
    }   
}

//Printing the path
void print_path(AdjacencyList graph,vector<int> path)
{
    // If graph is empty then return
    if (graph.empty())
        return;

    // If path is empty then return
    if (path.empty())
        return;

    for(unsigned int i=0;i<path.size()-1;i++){
        unsigned int node1 = path.at(i);
        unsigned int node2 = path.at(i+1);
        std::cout << "[" << node1 << "] to " ;
        for(unsigned int j=0;j<graph.at(node1).size();j++){
            if(graph.at(node1).at(j).first==node2){
                cout << "[" << graph.at(node1).at(j).first << "] = " << 
                            graph.at(node1).at(j).second << endl;
                break;
            }
        }
    }   
}


vector<int> brute_force_tsp(AdjacencyList& graph, int start) {

<<<<<<< HEAD
    // Completed in the Start folder
=======
    vector<int> order;
    //Let's list the nodes in a vector, and we can do permutations of
    //it as all of the nodes are connected
    vector<int> nodes;
    for (unsigned int i=0;i< graph.size();i++){
        nodes.push_back(i);
    }

    //We save the total path as a very large value (default)
    double minDistance = 1e6;

    //This while loop creates all possible permutations of the nodes
    //We can use this while loop to create an order of ID's visited
    //Let's look for te total path to visit nodes in current node order 
    do
    {
        bool OK=true;//We will use this to abolish search if needed
        unsigned int idx=1;//Let's start from index 1 to end 
        double dist=0; // Current distance that we have travelled throug nodes
        while((idx<nodes.size())&&(OK)){
            //We have two nodes 
            unsigned int node1 = nodes.at(idx-1);
            unsigned int node2 = nodes.at(idx);
            //We find in adjacency list node 1 connection to node2 and second element
            //in teh pair is the distance between these nodes
            dist+=graph.at(node1).at(node2).second;
            // std::cout << dist << " "; // This printed distance in debug mode
            //we can abolish search if we are already over the min distance
            if(dist>minDistance){
                OK=false;
            }
            idx++; // Otherwise we increment to next node
        }
        std::cout << std::endl;
        if(dist<minDistance){
            minDistance=dist; // Save minimum distance
            order.clear(); // clear the current order of nodes
            order=nodes; // Save the order of nodes
        }
    } 
    while (std::next_permutation(nodes.begin(), nodes.end()));

    return order;
>>>>>>> origin/subject
}


int main () {
    // Build the graph - Added here self connections to make it easier for our search

    AdjacencyList graph = {
        {{0,0},{1,50},{2,80},{3,40}}, // Node 0 is connected to nodes 0,1,2 and 3 (via edges with weights)
        {{0,50},{1,0},{2,10},{3,100}},     // Node 1 is connected to nodes 0,1,2 and 3 (via edges)
        {{0,80},{1,10},{2,0},{3,20}},     // Node 2 is connected to ....
        {{0,40},{1,100},{2,20},{3,0}},     // Node 3 is connected to ....
    };

    vector<int> path;

    cout << "Adjacency list: " << endl;
    print_adjacency_list(graph);

    path = brute_force_tsp(graph, 0);
    cout << "Path found by tsp search: " << endl;
    print_path(graph,path);

    return 0;
}
