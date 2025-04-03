#include <iostream>
#include <vector>
#include <algorithm>
#include <cmath>

#include "quadcopter.h"
#include "ros/ros.h"

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
   * The third argument below "week10_quad" is the rosnode name
   * The name must be unique, only one node of the same name can ever register with the roscore
   * If a rosnode with same name exists, it will be terminated
   */
  ros::init(argc, argv, "week10_quad");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle nh;

  /**
   * Create an object of type PfmsSample  and pass it a node handle
   */
  std::shared_ptr<Quadcopter> QuadPtr(new Quadcopter(nh));


  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   * 
   * However, we do not have a thread yet here (just because A2 is still being produced by students)
   * You do need to consider this and set it up so we can call ros::spin
   */

  ros::Rate loop_rate(10);
  bool reached = false; // we will break when reached

  while (ros::ok())
  {
    ros::spinOnce();

    reached = QuadPtr->reachGoal();
    if(reached){
        ROS_INFO_STREAM("We reached the goal");
    }

    loop_rate.sleep();
  }

  return 0;
}
