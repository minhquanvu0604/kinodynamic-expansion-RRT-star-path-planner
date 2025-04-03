Week 5 Tutorial Questions
=========================
Work through these questions and make sure you understand what is going on in each example. If you have any questions about the material, please raise them in the next tutorial session.

Implement a Graph: Adjacency List - Ex01
--------------------

In computer science, a Graph is a data structure that can be used to represent an interconnected network of objects.
Graphs consist of vertices and edges (which connect vertices).

In the skeleton supplied 
* An appropriate C++ data structure (vector of vectors) is used to represent an unweighted, undirected graph as an adjacency list 
* The `typedef` keyword to declare a type alias for this more complex type.
* A section commencing encoding on the graph below has commenced (the vector contains nodes and sub-vector is connectivity)

Complete the program so it:
* Encodes the pictured graph using the C++ data structure
* Prints the contents of the graph to screen

<img src="https://image.ibb.co/fL1A6U/example_graph.png" alt="sample graph to encode" width="200px"/>

Now consider the following:
* Draw a simplified floor plan of your house or make one up
* All rooms should be sealed off
* Mark the doorways and connections between rooms with a symbol
* Draw a graph where:
   * Vertices represent rooms
   * Edges represent connections

<img src="https://image.ibb.co/gCUCLp/week_05_graph_house.png" alt="sample map to encode" width="200px"/>

**Learning outcomes**: You should be able to create a graph, using edges and nodes to encode information, transferring a problem into a graph or tree. 

Breadth-First Search - Ex02
-----------------------------------------

Implement a function which:
* accepts as arguments:
   * A graph/adjacency list
   * A start and end vertex id
* Uses the *breadth-first search* algorithm to compute the shortest (least graph hops) path from start to end

Write a program which:
* Encodes your house as a graph
* Calculates and prints the shortest (least graph hops) path between two user-specified rooms

**Learning outcomes**: Familiarity with the use of set, map and queue to encode information. You can create an algorithm to search a graph trying to find a path between two nodes on the graph using BFS algorithm.

Depth First Search - Ex03
-------------------------

Copy your breadth-first search function and modify it to perform a *depth-first search*
* Does it still find the least graph hops path?
* What is the advantage of the depth-first search?

Now consider that in a real house it will take longer to move between some rooms than others
* How would you represent this?
* Can you find the true shortest path in terms of distance/time (not just the number of hops)?

What other real-world concepts could you represent with a graph?

**Learning outcomes**: Familiarity with the use of set, map and stack to encode information. You can create an algorithm to search a graph trying to find a path between two nodes on the graph using DFS algorithm.
