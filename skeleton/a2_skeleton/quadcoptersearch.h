#ifndef QUADCOPTERSEARCH_H
#define QUADCOPTERSEARCH_H

#include <iostream>
#include <vector>

#include "pfms_types.h"
#include "controllerinterface.h"
#include "graphsearch.h"


class QuadcopterSearch : public GraphSearch
{
public:

    /**
     * @brief Default constructor
     * 
     * The default constructors set necessary member variables to a default value 
    */
    QuadcopterSearch();

    /**
     * @brief Overloaded constructor
     * 
     * The overloaded constructors takes in the pointer to ControllerInterface object, in this context it is a pointer to 
     * Quadcopter object and is passed to the member variable
     * 
     * @param quadcopterPtr pointer to ControllerInterface object, which in this context is the Quadcopter
    */
    QuadcopterSearch(ControllerInterface* quadcopterPtr);

    /**
     * @brief Set pointer to ControllerInterface object
     * 
     * This is necessary to conduct graphsearch and is used only when the object is constructed with default contrustor
     * 
     * @param quadcopterPtr pointer to ControllerInterface object, which in this context is the Quadcopter
    */
    void setPtr(ControllerInterface* quadcopterPtr);

    /**
     * @brief Implementing Quadcopter graph search and set member variables regarding the optimal path order (vector of inteter)
     * and the minimum distance found
    */
    bool graphSearch() override;

    /**
     * @brief Get the optimal path 
     * 
     * This should be called after the call the graph search to return the most optimal path as a vector of integer
    */
    std::vector<int> getBestPath();

private:

    /**
     * @brief Pointer to ControllerInterface object
     * 
     * This is used to get the odometry of the quadcopter, which is necessary to find the initial position to conduct graph search
    */
    ControllerInterface* quadcopterPtr_; 

    /// @brief Best path order of the graph search
    std::vector<int> bestPath_;
};


#endif
