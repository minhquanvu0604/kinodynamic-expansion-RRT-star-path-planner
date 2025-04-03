#ifndef SUPERSEARCH_H
#define SUPERSEARCH_H

#include <iostream>
#include <vector>
#include <limits>
#include <cmath>
#include <stack>
#include <chrono>

#include "pfms_types.h"
#include "controllerinterface.h"
#include "graphsearch.h"


class SuperSearch : public GraphSearch
{
public:

    /**
     * @brief Default constructor
     * 
     * The default constructors set necessary member variables to a default value 
    */
    SuperSearch();


    
    SuperSearch(ControllerInterface* ackermanPtr, ControllerInterface* quadcopterPtr);


    void setPtrs(ControllerInterface* ackermanPtr, ControllerInterface* quadcopterPtr);
    
    void setGoals(std::vector<pfms::geometry_msgs::Point> goals) override;

    bool graphSearch() override;

    std::vector<int> getBestOrderAckerman();

    std::vector<int> getBestOrderQuadcopter();

private:

    ControllerInterface* ackermanPtr_; 
    ControllerInterface* quadcopterPtr_; 

    std::vector<int> bestOrderAckerman_;
    std::vector<int> bestOrderQuadcopter_;

    bool reorder(const std::vector<int>& local, std::vector<int>& global);

    void printVector(const std::vector<bool>& v);

    bool nextPermutation(std::vector<bool>& v);
};


#endif