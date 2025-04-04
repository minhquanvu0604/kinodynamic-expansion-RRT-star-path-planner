
#include "ros/ros.h"
#include "sample.h"


int main(int argc, char **argv)
{

  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line. For programmatic
   * remappings you can use a different version of init() which takes remappings
   * directly, but for most command-line programs, passing argc and argv is the easiest
   * way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   *
   * The third argument below "week10" is the rosnode name
   * The name must be unique, only one node of the same name can ever register with the roscore
   * If a rosnode with same name exists, it will be terminated
   */
  ros::init(argc, argv, "week10_laser");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle nh;

  /**
   * Create an object of type PfmsSample  and pass it a node handle
   */
  std::shared_ptr<PfmsSample> pfmsSamplePtr(new PfmsSample(nh));

  /**
   * Let's start seperate thread first, to do that we need to create object
   * and thereafter start the thread on teh function desired
   */
  std::thread t(&PfmsSample::seperateThread,pfmsSamplePtr);

  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
  ros::spin();

  /**
   * Let's cleanup everything, shutdown ros and join the thread
   */
  ros::shutdown();

  t.join();

  return 0;
}

