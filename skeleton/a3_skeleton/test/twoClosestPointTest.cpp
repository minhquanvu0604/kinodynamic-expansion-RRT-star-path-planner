#include <gtest/gtest.h>
// #include <climits>
#include <vector>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/Pose.h>
#include "laserprocessing.h"


// TEST (FirstCase, Test4){
    
//     // Set pose 
//     geometry_msgs::Pose pose;
//     pose.position.x = -0.806782;
//     pose.position.y = 50.748185;
//     pose.orientation = tf::createQuaternionMsgFromYaw(0.013834);

//     // geometry_msgs::Point posePoint;
//     // posePoint.x = pose.position.x;
//     // posePoint.y = pose.position.y;

//     // Set points detected
//     std::vector<geometry_msgs::Point> pointsDetected;

//     LaserProcessing process;
    
//     // Cone Y25
//     geometry_msgs::Point p0; //< Nearest 
//     p0.x = 13.123899;
//     p0.y = 55.210697;
//     p0.z = 0.55995;
//     // auto p0x = process.euclideanDistance(p0, posePoint);
//     // std::cout << "Distance from p0: " << p0x << std::endl;

//     // Cone B21
//     geometry_msgs::Point p1; //< Nearest 
//     p1.x = 13.153;
//     p1.y = 47.331;
//     p1.z = 0.55997;
//     // auto p1x = process.euclideanDistance(p1, posePoint);
//     // std::cout << "Distance from p1: " << p1x << std::endl;

//     // Cone Y24
//     geometry_msgs::Point p2;
//     p2.x = 24.478;
//     p2.y = 54.575;
//     p2.z = 0.559;

//     // Cone B22
//     geometry_msgs::Point p3;
//     p3.x = 24.541;
//     p3.y = 46.218;
//     p3.z = 0.56;

//     // Cone Y23
//     geometry_msgs::Point p4;
//     p4.x = 33.535;
//     p4.y = 48.051;
//     p4.z = 0.55995;

//     // Cone B23
//     geometry_msgs::Point p5;
//     p5.x = 22.109;
//     p5.y = 38.802;
//     p5.z = 0.56;


//     // Add points in random orders
//     pointsDetected.push_back(p3); //< Next nearest 
//     pointsDetected.push_back(p4);
//     pointsDetected.push_back(p1); //< Nearest 
//     pointsDetected.push_back(p2); //< Next nearest 
//     pointsDetected.push_back(p5);
//     pointsDetected.push_back(p0); //< Nearest 

//     // Test 2 nearest points
//     std::vector<unsigned int> twoNearestPoints;
//     bool OK = process.twoNearestPoints(pointsDetected, twoNearestPoints, pose);
//     auto processedPoints = twoNearestPoints;
    
//     EXPECT_TRUE(OK);

//     // DEBUG
//     for (auto i : twoNearestPoints){
//         std::cout << i << std::endl;
//     }
//     // Returns: 2 5
//     EXPECT_EQ(twoNearestPoints.front(), 2);
//     EXPECT_EQ(twoNearestPoints.back(), 5);


//     // // Test the next 2
//     // auto nextTwoNearestPoints = process.nextTwoNearestPoints(pointsDetected, twoNearestPoints, processedPoints);
//     // for (auto i : nextTwoNearestPoints){
//     //     std::cout << i << std::endl;
//     // }
//     // // Returns: 0 3
//     // EXPECT_EQ(nextTwoNearestPoints.front(), 0);
//     // EXPECT_EQ(nextTwoNearestPoints.back(), 3);
// }


// // TEST (SecondCase, Test4){

// //     // Set pose 
// //     geometry_msgs::Pose pose;
// //     pose.position.x = 10.510656;
// //     pose.position.y = 50.911408;
// //     geometry_msgs::Point posePoint;
// //     posePoint.x = pose.position.x;
// //     posePoint.y = pose.position.y;


// //     tf2::Quaternion q;
// //     q.setRPY(0, 0, 0.039283);
// //     pose.orientation.w = q.w();

// //     // Set points detected
// //     std::vector<geometry_msgs::Point> pointsDetected;

// //     LaserProcessing process;
    
// //     // Cone Y24
// //     geometry_msgs::Point p0; //< Nearest left
// //     p0.x = 24.599791;
// //     p0.y = 54.569763;
// //     p0.z = 0.55995;
// //     // auto p0x = process.euclideanDistance(p0, posePoint);
// //     // std::cout << "Distance from p0: " << p0x << std::endl;

// //     // Cone B22
// //     geometry_msgs::Point p1; //< Nearest right
// //     p1.x = 24.622831;
// //     p1.y = 46.130177;
// //     p1.z = 0.55997;
// //     // auto p1x = process.euclideanDistance(p1, posePoint);
// //     // std::cout << "Distance from p1: " << p1x << std::endl;

// //     // Cone Y22
// //     geometry_msgs::Point p2;
// //     p2.x = 33.644436;
// //     p2.y = 36.918217;
// //     p2.z = 0.559;

// //     // Cone B23
// //     geometry_msgs::Point p3; //< Next nearest right
// //     p3.x = 22.176085;
// //     p3.y = 38.698788;
// //     p3.z = 0.56;

// //     // Cone Y21
// //     geometry_msgs::Point p4;
// //     p4.x = 24.243954;
// //     p4.y = 31.016804;
// //     p4.z = 0.55995;

// //     // Cone Y20
// //     geometry_msgs::Point p5; ////////
// //     p5.x = 17.371977;
// //     p5.y = 23.12092;
// //     p5.z = 0.56;

// //     // Cone Y23
// //     geometry_msgs::Point p6; //< Next nearest left
// //     p6.x = 33.558659;
// //     p6.y = 47.937515;
// //     p6.z = 0.56;


// //     // Add points in random orders
// //     pointsDetected.push_back(p3); //< Next nearest right
// //     pointsDetected.push_back(p4);
// //     pointsDetected.push_back(p1); //< Nearest right
// //     pointsDetected.push_back(p2); 
// //     pointsDetected.push_back(p5); ////////
// //     pointsDetected.push_back(p0); //< Nearest left
// //     pointsDetected.push_back(p6); //< Next nearest left

// //     ASSERT_EQ(pointsDetected.size(), 7);

// //     // Test 2 nearest points
// //     auto twoNearestPoints = process.twoNearestPoints(pointsDetected, pose);

// //     // // DEBUG
// //     // std::cout << "TWO: " << std::endl;
// //     // for (auto i : twoNearestPoints){
// //     //     std::cout << i << std::endl;
// //     // }

// //     EXPECT_EQ(twoNearestPoints.at(0), 5); //< Nearest left 
// //     EXPECT_EQ(twoNearestPoints.at(1), 2); //< Nearest right

// //     // Test next 2 nearest points
// //     auto nextTwoNearestPoints = process.nextTwoNearestPoints(pointsDetected, twoNearestPoints);

// //     // // DEBUG
// //     // std::cout << "NEXT TWO: " << std::endl;
// //     // for (auto i : nextTwoNearestPoints){
// //     //     std::cout << i << std::endl;
// //     // }

// //     EXPECT_EQ(nextTwoNearestPoints.at(0), 6); //< Next nearest left 
// //     EXPECT_EQ(nextTwoNearestPoints.at(1), 0); //< Next nearest right

// // }

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
