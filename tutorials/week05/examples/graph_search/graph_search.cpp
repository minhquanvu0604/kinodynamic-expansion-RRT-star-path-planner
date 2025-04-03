#include <iostream>
#include <vector>
#include <stack>
#include <set>
#include <queue>
#include <map>
#include <algorithm>

using namespace std;

typedef vector<vector<int> > AdjacencyList;

//Printing the stack
void printStack(stack<int> s)
{
    // If stack is empty then return
    if (s.empty())
        return;


    int x = s.top();

    // Pop the top element of the stack
    s.pop();

    // Recursively call the function PrintStack
    printStack(s);

    // Print the stack element starting
    // from the bottom
    cout << x << " ";

    // Push the same element onto the stack
    // to preserve the order
    s.push(x);
}


vector<int> reconstruct_path(map<int,int> parents, int goal) {
    vector<int> path;
    path.push_back(goal);
    while (parents.find(path.back()) != parents.end()) {
        path.push_back(parents.at(path.back()) );
    }
    std::reverse(path.begin(), path.end());
    return path;
}

void print_path(vector<int> path) {
    for (vector<int>::iterator n = path.begin();
         n != path.end(); n++) {
        cout << *n << " ";
    }
    cout << endl;
}


vector<int> breadth_first_search(AdjacencyList& graph, int start, int goal) {
    cout << "Breadth first search exploration order: ";
    // Additional search variables
    int current;
    map<int,int> parents;
    // create empty set S
    set<int> visited;
    // create empty queue Q
    queue<int> q;
    // add root to S
    visited.insert(start);
    // Q.enqueue(root)
    q.push(start);
    // while Q is not empty:
    while (!q.empty()) {
        // current = Q.dequeue()
        current = q.front();
        q.pop();
        // Printing exploration order
        cout << current << " ";
        // if current is the goal:
        if (current == goal) {
            // return current
            cout << endl;
            return reconstruct_path(parents, current); // with the ultimate map of parents, reconstruct the path
        }
        // for each node n that is adjacent to current:
        for (vector<int>::iterator n = graph.at(current).begin();
             n != graph.at(current).end(); n++) {
            // if n is not in S:
            if (visited.find(*n) == visited.end()) {
                // add n to S
                visited.insert(*n);
                // n.parent = current
                parents[*n] = current;
                // Q.enqueue(n)
                q.push(*n);
            }
        }
    }
    vector<int> empty_path={};
    return empty_path;
}

vector<int> depth_first_search(AdjacencyList& graph, int root, int goal) {
    cout << "Depth first search exploration order: ";
    // Additional search variables
    int current;
    map<int,int> parents;

    // create empty set S
    set<int> visted;

    // create empty queue Q
    stack<int> q;
    // add root to S
    visted.insert(root);
    // Q.enqueue(root)
    q.push(root);
    // while Q is not empty:
    while (!q.empty()) {
        // Go through the nodes 

        // current = Q.dequeue()
        current = q.top(); // integer for node id
        // Printing exploration order
        cout << current << " ";
        // if current is the goal:
        if (current == goal) {
            // return current
            cout << endl;

            // with the ultimate map of parents, reconstruct the path
            return reconstruct_path(parents, current); 
        }
        // for each node n that is adjacent to current:
        bool unvisited=true;

        // Go through the neighbors of the current node
        for (vector<int>::iterator n = graph.at(current).begin(); // graph is vector< vector<int> >
             n != graph.at(current).end(); n++) {
            // if n visted not in S:
            if (visted.find(*n) == visted.end()) {
                // add n to S
                visted.insert(*n);

                // sets the parent of the neighbor *n to the current node
                // n.parent = current
                parents[*n] = current; // overwritten until found the goal -> ultimate parents map

                // Q.enqueue(n)
                q.push(*n);//We add node to stack
                unvisited = false;// If we can visit it means we do not pop from stack
                break;//Once we add a node we break here
            }
        }
        if (unvisited){//If we have not visited any node we pop it from stack
            q.pop();
        }
    }
    vector<int> empty_path={};
    return empty_path;
}


int main () {
    // Build the graph
    AdjacencyList house_graph = {
        {1, 2},     // Node 0 is connected to nodes 1 and 2 (via edges)
        {0, 4},     // Node 1 is connected to nodes 0 and 4 (via edges)
        {0, 3, 4},  // Node 2 is connected to nodes 0, 3 and 4 (via edges)
        {2, 4, 5},  // Node 3
        {1, 2, 3},  // Node 4
        {3, 6},     // Node 5
        {5}         // Node 6
    };

    vector<int> path;

    path = breadth_first_search(house_graph, 5, 1);
    cout << "Path found by breadth first search: ";
    print_path(path);

    path = depth_first_search(house_graph, 5, 1);
    cout << "Path found by depth first search: ";
    print_path(path);

    return 0;
}
