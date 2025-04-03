#include "gtest/gtest.h"
#include <iostream>
#include <vector>
#include <cmath>
#include <tf2/LinearMath/Quaternion.h>

#include <ros/package.h> //This tool allows to identify the path of the package on your system
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include "ros/ros.h"

//Student defined libraries
#include "ackerman.h"
#include "heuristicRRTstar/heuristicRRTstar.h"


using namespace std;
using namespace nav_msgs;

TEST(Planning,Planning1){

    //! The below code tests the laserprocessing class
    //! The data has been saved in a bag, that is opened and used.
    //! Unforttunately as we need to use a ConstPtr below, we can't make this
    //! a helper function

    //! Below command allows to find the folder belonging to a package
    std::string path = ros::package::getPath("ugv");
    // Now we have the path, the images for our testing are stored in a subfolder /test/samples
    path += "/test/bag/";
    std::string file = path + "pathplanning1.bag";

    //! Manipulating rosbag, from: http://wiki.ros.org/rosbag/Code%20API
    rosbag::Bag bag;
    bag.open(file);  // BagMode is Read by default
    sensor_msgs::LaserScan::ConstPtr laserScan = nullptr;
    nav_msgs::Odometry::ConstPtr odom = nullptr;

    for(rosbag::MessageInstance const m: rosbag::View(bag))
    {
        if(m.getTopic() == "/orange/laser/scan"){
        if( laserScan == nullptr){
            laserScan = m.instantiate<sensor_msgs::LaserScan>();
        }
        }
        if(m.getTopic() == "/ugv_odom"){
        if( odom == nullptr){
            odom = m.instantiate<nav_msgs::Odometry>();
        }
        }
        if ((laserScan != nullptr) && (odom != nullptr)){
        //! Now we have a laserScan and odometry so we can proceed
        //! We could also check here if we have High Intensity readings before abandoning the loop
        break;
        }
    }
    bag.close();

    ASSERT_NE(laserScan, nullptr);//Check that we have a laser scan from the bag
    ASSERT_NE(odom, nullptr);//Check that we have a laser scan from the bag


    nav_msgs::Odometry odo = *odom;
    geometry_msgs::Pose pose = odo.pose.pose;
    Point ackermanPosition(pose.position.x, pose.position.y);
    std::atomic_bool running(false);

    LaserProcessing process(*laserScan);

    // Get goal points
    auto conesInd = process.conePointIndex(&running);
    auto cones = process.getPoints(conesInd, pose);

    Node start;
    start.x = pose.position.x;
    start.y = pose.position.y;
    start.yaw = tf::getYaw(pose.orientation);


    ros::NodeHandle nh;
    long id = 0;
    RRTStar rrt(nh, id);
    rrt.setData(start, cones, cones);
    rrt.createTree();

    auto bestPath = rrt.chooseBestPath();

    EXPECT_NEAR(bestPath.at(3).x,-9.591413, 4);
    EXPECT_NEAR(bestPath.at(3).y,-48.368954, 4);

}

TEST(Planning,Planning2){

    //! The below code tests the laserprocessing class
    //! The data has been saved in a bag, that is opened and used.
    //! Unforttunately as we need to use a ConstPtr below, we can't make this
    //! a helper function

    //! Below command allows to find the folder belonging to a package
    std::string path = ros::package::getPath("ugv");
    // Now we have the path, the images for our testing are stored in a subfolder /test/samples
    path += "/test/bag/";
    std::string file = path + "pathplanning2.bag";

    //! Manipulating rosbag, from: http://wiki.ros.org/rosbag/Code%20API
    rosbag::Bag bag;
    bag.open(file);  // BagMode is Read by default
    sensor_msgs::LaserScan::ConstPtr laserScan = nullptr;
    nav_msgs::Odometry::ConstPtr odom = nullptr;

    for(rosbag::MessageInstance const m: rosbag::View(bag))
    {
        if(m.getTopic() == "/orange/laser/scan"){
        if( laserScan == nullptr){
            laserScan = m.instantiate<sensor_msgs::LaserScan>();
        }
        }
        if(m.getTopic() == "/ugv_odom"){
        if( odom == nullptr){
            odom = m.instantiate<nav_msgs::Odometry>();
        }
        }
        if ((laserScan != nullptr) && (odom != nullptr)){
        //! Now we have a laserScan and odometry so we can proceed
        //! We could also check here if we have High Intensity readings before abandoning the loop
        break;
        }
    }
    bag.close();

    ASSERT_NE(laserScan, nullptr);//Check that we have a laser scan from the bag
    ASSERT_NE(odom, nullptr);//Check that we have a laser scan from the bag


    nav_msgs::Odometry odo = *odom;
    geometry_msgs::Pose pose = odo.pose.pose;
    Point ackermanPosition(pose.position.x, pose.position.y);
    std::atomic_bool running(false);

    LaserProcessing process(*laserScan);

    // Get goal points
    auto conesInd = process.conePointIndex(&running);
    auto cones = process.getPoints(conesInd, pose);

    Node start;
    start.x = pose.position.x;
    start.y = pose.position.y;
    start.yaw = tf::getYaw(pose.orientation);


    ros::NodeHandle nh;
    long id = 0;
    RRTStar rrt(nh, id);
    rrt.setData(start, cones, cones);
    rrt.createTree();

    auto bestPath = rrt.chooseBestPath();

    EXPECT_NEAR(bestPath.at(3).x,-27.4072, 4);
    EXPECT_NEAR(bestPath.at(3).y,-48.4784, 4);

}

TEST(Planning,Planning3){

    //! The below code tests the laserprocessing class
    //! The data has been saved in a bag, that is opened and used.
    //! Unforttunately as we need to use a ConstPtr below, we can't make this
    //! a helper function

    //! Below command allows to find the folder belonging to a package
    std::string path = ros::package::getPath("ugv");
    // Now we have the path, the images for our testing are stored in a subfolder /test/samples
    path += "/test/bag/";
    std::string file = path + "pathplanning3.bag";

    //! Manipulating rosbag, from: http://wiki.ros.org/rosbag/Code%20API
    rosbag::Bag bag;
    bag.open(file);  // BagMode is Read by default
    sensor_msgs::LaserScan::ConstPtr laserScan = nullptr;
    nav_msgs::Odometry::ConstPtr odom = nullptr;

    for(rosbag::MessageInstance const m: rosbag::View(bag))
    {
        if(m.getTopic() == "/orange/laser/scan"){
        if( laserScan == nullptr){
            laserScan = m.instantiate<sensor_msgs::LaserScan>();
        }
        }
        if(m.getTopic() == "/ugv_odom"){
        if( odom == nullptr){
            odom = m.instantiate<nav_msgs::Odometry>();
        }
        }
        if ((laserScan != nullptr) && (odom != nullptr)){
        //! Now we have a laserScan and odometry so we can proceed
        //! We could also check here if we have High Intensity readings before abandoning the loop
        break;
        }
    }
    bag.close();

    ASSERT_NE(laserScan, nullptr);//Check that we have a laser scan from the bag
    ASSERT_NE(odom, nullptr);//Check that we have a laser scan from the bag


    nav_msgs::Odometry odo = *odom;
    geometry_msgs::Pose pose = odo.pose.pose;
    Point ackermanPosition(pose.position.x, pose.position.y);
    std::atomic_bool running(false);

    LaserProcessing process(*laserScan);

    // Get goal points
    auto conesInd = process.conePointIndex(&running);
    auto cones = process.getPoints(conesInd, pose);

    Node start;
    start.x = pose.position.x;
    start.y = pose.position.y;
    start.yaw = tf::getYaw(pose.orientation);


    ros::NodeHandle nh;
    long id = 0;
    RRTStar rrt(nh, id);
    rrt.setData(start, cones, cones);
    rrt.createTree();

    auto bestPath = rrt.chooseBestPath();

    EXPECT_NEAR(bestPath.at(3).x,-50.1693, 4);
    EXPECT_NEAR(bestPath.at(3).y,-18.0339, 4);
}



int main(int argc, char **argv) {
  ros::init(argc, argv, "test_node");
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
