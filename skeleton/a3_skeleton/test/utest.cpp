#include <gtest/gtest.h>
#include <climits>
#include <vector>

#include <ros/package.h> //This tool allows to identify the path of the package on your system
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include "tf/transform_datatypes.h" //To use getYaw function from the quaternion of orientation

#include "laserprocessing.h"


TEST(LaserProcessing,TestClosestPosition){

  //! The below code tests the laserprocessing class
  //! The data has been saved in a bag, that is opened and used.
  //! Unforttunately as we need to use a ConstPtr below, we can't make this
  //! a helper function

  //! Below command allows to find the folder belonging to a package
  std::string path = ros::package::getPath("a3_skeleton");
  // Now we have the path, the images for our testing are stored in a subfolder /test/samples
  path += "/test/bag/";
  std::string file = path + "dome.bag";

  //! Manipulating rosbag, from: http://wiki.ros.org/rosbag/Code%20API
  rosbag::Bag bag;
  bag.open(file);  // BagMode is Read by default
  sensor_msgs::LaserScan::ConstPtr laserScan = nullptr;
  nav_msgs::Odometry::ConstPtr odom = nullptr;

  for(rosbag::MessageInstance const m: rosbag::View(bag))
  {
    if(m.getTopic() == "/robot_0/base_scan"){
      if( laserScan == nullptr){
        laserScan = m.instantiate<sensor_msgs::LaserScan>();
      }
    }
    if(m.getTopic() == "/robot_0/odom"){
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

  ////////////////////////////////////////////
  // Our code is tested below


  //! Create an object of LaserProcessing class as we will use the public function of that object to run tests against
  // LaserProcessing laserProcessing(*laserScan);
  // geometry_msgs::Point point = laserProcessing.detectPositionClosest();

  // We check the closest point knowing that this is indeed the actual value, refer to quiz 5 for more examples
  // EXPECT_NEAR(point.x,0.0,0.01);
  // EXPECT_NEAR(point.y,4.12,0.01);

}


int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
