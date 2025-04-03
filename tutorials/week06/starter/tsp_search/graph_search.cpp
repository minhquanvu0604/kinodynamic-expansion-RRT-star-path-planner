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
        std::cout << "[" << nodeNum++ << "]" << std::endl ;
        for(auto nodes : edges){
            std::cout << nodes.first << ":" << nodes.second << " ";
        }
        std::cout << std::endl ;
    }   
    
    cout << endl;
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
    
    vector<int> order;
    double longestPath;

    //Let's list the nodes in a vector, and we can do permutations of
    //it as all of the nodes are connected
    vector<int> nodes;
    for (unsigned int i=0;i< graph.size();i++){
        nodes.push_back(i);
    }

    double minDistance = std::numeric_limits<double>::max();

    //This while loop creates all possible permutations of the nodes
    //We can use this while loop to create an order of ID's visited
    //Let's look for the total path to visit nodes in current node order 
    do
    {
        // Pruning
        bool nextPermut = false;

        // Don't need to go through all permuts given the starting point
        if (nodes.front() != start) continue;

        // Print permut
        for (auto node : nodes){
            cout << node << " ";
        }
        cout << " ";


        double totalDistance = 0;

        // Loop over the nodes ID of the current permutation
        for (unsigned int i = 0; i < nodes.size()-1; i++){
            // For every edge of that node
            for (auto edge : graph.at(nodes.at(i)))
                if (nodes.at(i+1) == edge.first){
                    totalDistance += edge.second;

                    if (totalDistance > minDistance){
                        nextPermut = true;
                        break;
                    }
                }
            if (nextPermut) break;                
        }
        
        // Pruning
        if (nextPermut) {
            cout << endl;
            continue;
        }
        
        if (totalDistance < minDistance) {
            minDistance = totalDistance;
            order = nodes;
        }
    
        cout << "Total Distance: " << totalDistance << endl;
    } 
    while (std::next_permutation(nodes.begin(), nodes.end()));

    // What is the shortest length? 
    // What is the order associated to this shortest length?

    return order;
}

vector<int> brute_force_tsp_improved(AdjacencyList& graph, int start) {
    int current = start;
    

}


// vector<int> tsp_search(AdjacencyList& graph, int start) {
//     cout << "TSP exploration order: ";
//     // Additional search variables
//     int current;
//     map<int,int> parents;
//     // create empty set S
//     set<int> visited;
//     // create empty queue Q
//     queue<int> q;
//     // add root to S
//     visited.insert(start);
//     // Q.enqueue(root)
//     q.push(start);
//     // while Q is not empty:
//     while (!q.empty()) {
//         // current = Q.dequeue()
//         current = q.front();
//         q.pop();
//         // Printing exploration order
//         cout << current << " ";
//         // if current is the goal:
//         if (current == goal) {
//             // return current
//             cout << endl;
//             return reconstruct_path(parents, current);
//         }
//         // for each node n that is adjacent to current:
//         for (vector<int>::iterator n = graph.at(current).begin();
//              n != graph.at(current).end(); n++) {
//             // if n is not in S:
//             if (visited.find(*n) == visited.end()) {
//                 // add n to S
//                 visited.insert(*n);
//                 // n.parent = current
//                 parents[*n] = current;
//                 // Q.enqueue(n)
//                 q.push(*n);
//             }
//         }
//     }
//     vector<int> empty_path={};
//     return empty_path;
// }

int main () {
    // Build the graph
    AdjacencyList graph = {
        {{1,50},{2,80},{3,40}}, // Node 0 is connected to nodes 1,2 and 3 (via edges with weights)
        {{0,50},{2,10},{3,100}},     // Node 1 is connected to nodes 0,2 and 3 (via edges)
        {{0,80},{1,10},{3,20}},     // Node 2 is connected to ....
        {{0,40},{1,100},{2,20}},     // Node 3 is connected to ....
    };

    vector<int> path;

    cout << "Adjacency list: " << endl;
    print_adjacency_list(graph);

    path = brute_force_tsp(graph, 0);

    cout << "\n";
    cout << "Path of brute force: ";
    for (auto ele : path) cout << ele << " ";
    cout << endl;

    // cout << "Path found by depth first search: ";
    print_path(graph,path);

    return 0;
}
