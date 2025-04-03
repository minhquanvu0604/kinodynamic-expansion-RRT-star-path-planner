Week 6 Tutorial - STL Questions
=========================
Implement a TSP Brute Force Search
--------------------

In computer science, a Graph is a data structure that can be used to represent an interconnected network of objects. Graphs consist of vertices and edges (which connect vertices).

In the skeleton supplied an appropriate C++ data structure (vector of vectors of pairs) is used to represent an weighted, directed graph as an adjacency list 

```typedef vector<vector<pair<int,double>> > AdjacencyList```

We have the following graph, the goal is to search and obtain the shortest path to visit all nodes from a given starting node. This is a variant of the  [Travelling Salesman Problem (TSP)](https://en.wikipedia.org/wiki/Travelling_salesman_problem).

<img src="./images/tsp.png" alt="graph to encode" width="200px"/>

A section commencing encoding on the graph above has commenced (the vector contains nodes and sub-vector is connectivity - with edge and weights as a pair )

**TASK 1**

Complete the program so it encodes the pictured graph using the C++ data structure

**TASK 2**

The most naive approach to solve TSP is the "Brute Force Approach". Given that all the nodes are connected, the Brute Force Approach looks at all the possible permutations of visiting nodes `[0,1,2,3]   [0 1 3 2]  [0 2 1 3] etc` and analyses them for shortest path. We  generate all the permutations of the nodes using the `algorithm` from STL. 

Your task is now to go through the permutations, and determine which one has the shortest path (which is the search order we are after).

Consider:
* Do we need all the permutations if the start node is supplied, could we further limit this
* Do we need to compute a whole path across all nodes to know it is shortest or can we abandon the search earlier



