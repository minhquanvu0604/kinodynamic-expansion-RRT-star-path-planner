#include <gtest/gtest.h>
#include <climits>
#include <vector>

#include <ros/package.h> //This tool allows to identify the path of the package on your system
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <sensor_msgs/Range.h>
#include <nav_msgs/Odometry.h>

#include "../src/rangeprocessing.h"


TEST(RangeProcessing,DetectPerson){

  //! The below code tests the rangeprocessing class
  //! The data has been saved in a bag, that is opened and used.
  //! Unforttunately as we need to use a ConstPtr below, we can't make this
  //! a helper function

  //! Below command allows to find the folder belonging to a package
  std::string path = ros::package::getPath("quiz5_a2");
  // Now we have the path, the images for our testing are stored in a subfolder /test/samples
  path += "/test/bag/";
  std::string file = path + "sample2.bag";

  //! Manipulating rosbag, from: http://wiki.ros.org/rosbag/Code%20API
  rosbag::Bag bag;
  bag.open(file);  // BagMode is Read by default
  sensor_msgs::Range::ConstPtr range = nullptr;
  nav_msgs::Odometry::ConstPtr odom = nullptr;

  //! The bag has all the messages, so we go through all of them to find the mesages we need
  for(rosbag::MessageInstance const m: rosbag::View(bag))
  {
    //! We will go through the bag and extract the range scan and odometry
    //! We have to try to instatitate each message type

    if(m.getTopic() == "/drone/sonar/range"){
      if( range == nullptr){
        range = m.instantiate<sensor_msgs::Range>();
      }
    }
    if(m.getTopic() == "/uav_odom"){
      if( odom == nullptr){
        odom = m.instantiate<nav_msgs::Odometry>();
      }
    }
    if ((range != nullptr) && (odom != nullptr)){
      //! Now we have a range and odometry so we can proceed
      break;
    }
  }
  bag.close();


  ASSERT_NE(range, nullptr);//Check that we have a range scan from the bag
  ASSERT_NE(odom, nullptr);//Check that we have odo from the bag

  ////////////////////////////////////////////
  // Our code is tested below
  double personHeight=1.86;
  geometry_msgs::Point location;
  bool personDetected = range_processing::detectPerson(*range, odom->pose.pose, personHeight, location);


  EXPECT_TRUE(personDetected);

  EXPECT_NEAR(location.x,5.01,0.05);
  EXPECT_NEAR(location.y,6.30,0.05);
  EXPECT_NEAR(location.z,1.86,0.05);
}


int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
