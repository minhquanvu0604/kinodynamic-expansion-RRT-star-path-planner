#include "gtest/gtest.h"
#include <iostream>
#include <vector>
#include "pfms_types.h"
#include "supersearch.h"
#include "ackerman.h"
#include "quadcopter.h"

// Some helper header for assembling messages and testing
#include "test_helper.h"
#include "linkcommand.h"

TEST(SuperSearch, FirstTest){

    //The below teleports the platforms to starting location
    LinkCommand* linkCommand = new LinkCommand();
    {
        Odometry odo = populateOdoUGV(100,100,0);
        linkCommand->writeCommand(odo);
        odo = populateOdoUAV(-100,-100,2,0);
        linkCommand->writeCommand(odo);
    }

    std::vector<ControllerInterface*> controllers;
    controllers.push_back(new Ackerman());
    controllers.push_back(new Quadcopter());


    // It is obvious to find the sorted goals of ackerman 
    // This should returns:
    // Ackerman: 0 1 2
    // Quadcopter: 4 5 3
    std::vector<pfms::geometry_msgs::Point> goalsAckerman = {
        {50,100}, {100,50}, {50, 50}
    };

    std::vector<pfms::geometry_msgs::Point> goalsQuadcopter = {
        {-50,-100}, {-100,-50}, {-50, -50}
    };


    // Initialize GraphSearch object to conduct graph search 
    SuperSearch search(controllers.front(), controllers.back());
    
    // Set goals
    search.setGoals(goalsAckerman);
    search.setGoals(goalsQuadcopter);

    // Implement graph search
    search.graphSearch();

    // Show the best order
    std::cout << "Best goal order:" << std::endl;
    std::cout << "Ackerman: ";
    for (auto ele : search.getBestOrderAckerman())
        std::cout << ele << " ";
    std::cout << std::endl;

    std::cout << "Quadcopter: ";
    for (auto ele : search.getBestOrderQuadcopter())
        std::cout << ele << " ";
    std::cout << std::endl;

    // Time elapsed
    std::cout << "Time elapsed for the graph search: " << search.getTimeElapsed() << " milliseconds."<< std::endl;
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}

