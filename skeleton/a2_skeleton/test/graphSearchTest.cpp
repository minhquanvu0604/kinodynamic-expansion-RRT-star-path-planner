#include "gtest/gtest.h"
#include <iostream>
#include <vector>
#include "pfms_types.h"
#include "ackermansearch.h"
#include "quadcoptersearch.h"
#include "ackerman.h"
#include "quadcopter.h"

// Some helper header for assembling messages and testing
#include "test_helper.h"
#include "linkcommand.h"

///////////////////////////////////////////////////////////
// Unit Tests Start HERE
////////////////////////////////////////////////////////

// Ackerman Tests
TEST(AckermanGraphSearch, FirstTest){

    LinkCommand* linkCommand = new LinkCommand(true);
    {
        Odometry odo = populateOdoUGV(0,2,0);
        linkCommand->writeCommand(odo);
    }
    
    std::vector<ControllerInterface*> controllers;
    controllers.push_back(new Ackerman());

    std::vector<pfms::geometry_msgs::Point> goals = {
        {0, 10}, {0, 30}, {0, 20}, {0, 40}, {0, 50}, {0, 60}
    };

    AckermanSearch search;

    // Implement the graph search
    search.setPtr(controllers.front());
    search.setGoals(goals);
    bool OK = search.graphSearch();
    ASSERT_TRUE(OK);

    double minCost = search.getMinCost();
    std::vector<int> bestPath = search.getBestPath();
    
    // Time elapsed
    std::cout << "Time elapsed for the graph search: " << search.getTimeElapsed() << " milliseconds."<< std::endl;


    std::cout << "Min Distance: " << minCost << std::endl;
    std::cout << "Best Path: ";
    for (int i = 0; i < bestPath.size(); i++)
        std::cout << bestPath.at(i) << " ";
    std::cout << std::endl;

    ASSERT_EQ(bestPath.size(), goals.size());
    EXPECT_EQ(bestPath.at(0), 0);
    EXPECT_EQ(bestPath.at(1), 2);
    EXPECT_EQ(bestPath.at(2), 1);
    EXPECT_EQ(bestPath.at(3), 3);
    EXPECT_EQ(bestPath.at(4), 4);
    EXPECT_EQ(bestPath.at(5), 5);
}



TEST(AckermanGraphSearch, SecondTest){

    LinkCommand* linkCommand = new LinkCommand(true);
    {
        Odometry odo = populateOdoUGV(0,2,0);
        linkCommand->writeCommand(odo);
    }
    
    std::vector<ControllerInterface*> controllers;
    controllers.push_back(new Ackerman());

    std::vector<pfms::geometry_msgs::Point> goals = {
        {0, 10}, {0, 60}, {0, 50}, {0, 40}, {0, 30}, {0, 20}
    };

    AckermanSearch search;

    // Implement the graph search
    search.setPtr(controllers.front());
    search.setGoals(goals);
    bool OK = search.graphSearch();
    ASSERT_TRUE(OK);

    double minCost = search.getMinCost();
    std::vector<int> bestPath = search.getBestPath();
    
    // Time elapsed
    std::cout << "Time elapsed for the graph search: " << search.getTimeElapsed() << " milliseconds."<< std::endl;

    std::cout << "Min Distance: " << minCost << std::endl;
    std::cout << "Best Path: ";
    for (int i = 0; i < bestPath.size(); i++)
        std::cout << bestPath.at(i) << " ";
    std::cout << std::endl;

    ASSERT_EQ(bestPath.size(), goals.size());
    EXPECT_EQ(bestPath.at(0), 0);
    EXPECT_EQ(bestPath.at(1), 5);
    EXPECT_EQ(bestPath.at(2), 4);
    EXPECT_EQ(bestPath.at(3), 3);
    EXPECT_EQ(bestPath.at(4), 2);
    EXPECT_EQ(bestPath.at(5), 1);
}

// Quadcopter Tests
TEST(QuadcopterGraphSearch, FirstTest){

    LinkCommand* linkCommand = new LinkCommand(true);
    {
        Odometry odo = populateOdoUAV(0,5,2,0);
        linkCommand->writeCommand(odo);
    }
    
    std::vector<ControllerInterface*> controllers;
    controllers.push_back(new Quadcopter());

    std::vector<pfms::geometry_msgs::Point> goals = {
        {0, 5}, {10, -2}, {10, 6}, {0, -2}, {-3, -20}, {-10, -20}
    };

    QuadcopterSearch search;

    // Implement the graph search
    search.setPtr(controllers.front());
    search.setGoals(goals);
    bool OK = search.graphSearch();
    ASSERT_TRUE(OK);

    double minCost = search.getMinCost();
    std::vector<int> bestPath = search.getBestPath();
    
    // Time elapsed
    std::cout << "Time elapsed for the graph search: " << search.getTimeElapsed() << " milliseconds."<< std::endl;

    std::cout << "Min Distance: " << minCost << std::endl;
    std::cout << "Best Path: ";
    for (int i = 0; i < bestPath.size(); i++)
        std::cout << bestPath.at(i) << " ";
    std::cout << std::endl;


    ASSERT_EQ(bestPath.size(), goals.size());
    EXPECT_EQ(bestPath.at(0), 0);
    EXPECT_EQ(bestPath.at(1), 2);
    EXPECT_EQ(bestPath.at(2), 1);
    EXPECT_EQ(bestPath.at(3), 3);
    EXPECT_EQ(bestPath.at(4), 4);
    EXPECT_EQ(bestPath.at(5), 5);

    
}

TEST(QuadcopterGraphSearch, SecondTest){

    LinkCommand* linkCommand = new LinkCommand(true);
    {
        Odometry odo = populateOdoUAV(0,5,2,0);
        linkCommand->writeCommand(odo);
    }
    
    std::vector<ControllerInterface*> controllers;
    controllers.push_back(new Quadcopter());

    std::vector<pfms::geometry_msgs::Point> goals = {
        {0, 5}, {-10, 40}, {-17, 40}, {-20, 9}, {-15, -5}, {0, 2}
    };

    QuadcopterSearch search;

    // Implement the graph search
    search.setPtr(controllers.front());
    search.setGoals(goals);
    bool OK = search.graphSearch();
    ASSERT_TRUE(OK);

    double minCost = search.getMinCost();
    std::vector<int> bestPath = search.getBestPath();
    
    // Time elapsed
    std::cout << "Time elapsed for the graph search: " << search.getTimeElapsed() << " milliseconds."<< std::endl;


    std::cout << "Min Distance: " << minCost << std::endl;
    std::cout << "Best Path: ";
    for (int i = 0; i < bestPath.size(); i++)
        std::cout << bestPath.at(i) << " ";
    std::cout << std::endl;

    ASSERT_EQ(bestPath.size(), goals.size());
    EXPECT_EQ(bestPath.at(0), 0);
    EXPECT_EQ(bestPath.at(1), 5);
    EXPECT_EQ(bestPath.at(2), 4);
    EXPECT_EQ(bestPath.at(3), 3);
    EXPECT_EQ(bestPath.at(4), 2);
    EXPECT_EQ(bestPath.at(5), 1);
}

TEST(AckermanUnknownResult, OnlyTest){
    std::cout << "AckermanUnknownResult test:" << std::endl;

    LinkCommand* linkCommand = new LinkCommand(true);
    {
        Odometry odo = populateOdoUGV(0,2,0);
        linkCommand->writeCommand(odo);
    }
    
    std::vector<ControllerInterface*> controllers;
    controllers.push_back(new Ackerman());

    std::vector<pfms::geometry_msgs::Point> goals;

    //Goals
    pfms::geometry_msgs::Point goal0{8,-1};
    pfms::geometry_msgs::Point goal1{-5,-2};
    pfms::geometry_msgs::Point goal2{10,-2};
    pfms::geometry_msgs::Point goal3{-6,-4};
    pfms::geometry_msgs::Point goal4{4,5};
    pfms::geometry_msgs::Point goal5{10,20};
    pfms::geometry_msgs::Point goal6{7,2};

    goals.push_back(goal0);
    goals.push_back(goal1);
    goals.push_back(goal2);
    goals.push_back(goal3);
    goals.push_back(goal4);
    goals.push_back(goal5);
    goals.push_back(goal6);

    AckermanSearch search;

    // Implement the graph search
    search.setPtr(controllers.front());
    search.setGoals(goals);
    bool OK = search.graphSearch();
    ASSERT_TRUE(OK);

    double minCost = search.getMinCost();
    std::vector<int> bestPath = search.getBestPath();
    
    // Time elapsed
    std::cout << "Time elapsed for the graph search: " << search.getTimeElapsed() << " milliseconds."<< std::endl;

    std::cout << "Min Distance: " << minCost << std::endl;
    std::cout << "Best Path: ";
    for (int i = 0; i < bestPath.size(); i++)
        std::cout << bestPath.at(i) << " ";
    std::cout << std::endl;
}


int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}

