// // Ackerman Tests
// TEST(AckermanGraphSearch, FirstTest){

//     LinkCommand* linkCommand = new LinkCommand(true);
//     {
//         Odometry odo = populateOdoUGV(0,2,0);
//         linkCommand->writeCommand(odo);
//     }
    
//     std::vector<ControllerInterface*> controllers;
//     controllers.push_back(new Ackerman());

//     std::vector<pfms::geometry_msgs::Point> goals = {
//         {0, 10}, {0, 30}, {0, 20}, {0, 40}, {0, 50}, {0, 60}
//     };

//     auto result = graphSearchAckerman(goals, controllers.front());
//     auto bestPath = result.second;

    
//     std::cout << "Best Path: ";
//     for (int i = 0; i < bestPath.size(); i++)
//         std::cout << bestPath.at(i) << " ";
//     std::cout << std::endl;

//     ASSERT_EQ(bestPath.size(), goals.size());
//     EXPECT_EQ(bestPath.at(0), 0);
//     EXPECT_EQ(bestPath.at(1), 2);
//     EXPECT_EQ(bestPath.at(2), 1);
//     EXPECT_EQ(bestPath.at(3), 3);
//     EXPECT_EQ(bestPath.at(4), 4);
//     EXPECT_EQ(bestPath.at(5), 5);
// }

// TEST(AckermanGraphSearch, SecondTest){

//     LinkCommand* linkCommand = new LinkCommand(true);
//     {
//         Odometry odo = populateOdoUGV(0,2,0);
//         linkCommand->writeCommand(odo);
//     }
    
//     std::vector<ControllerInterface*> controllers;
//     controllers.push_back(new Ackerman());

//     std::vector<pfms::geometry_msgs::Point> goals = {
//         {0, 10}, {0, 60}, {0, 50}, {0, 40}, {0, 30}, {0, 20}
//     };

//     auto result = graphSearchAckerman(goals, controllers.front());
//     auto bestPath = result.second;

//     std::cout << "Best Path: ";
//     for (int i = 0; i < bestPath.size(); i++)
//         std::cout << bestPath.at(i) << " ";
//     std::cout << std::endl;

//     ASSERT_EQ(bestPath.size(), goals.size());
//     EXPECT_EQ(bestPath.at(0), 0);
//     EXPECT_EQ(bestPath.at(1), 5);
//     EXPECT_EQ(bestPath.at(2), 4);
//     EXPECT_EQ(bestPath.at(3), 3);
//     EXPECT_EQ(bestPath.at(4), 2);
//     EXPECT_EQ(bestPath.at(5), 1);
// }
