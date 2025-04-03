#include "ros/ros.h"
//#include "std_msgs/String.h"
#include "std_msgs/Float64.h"

#include <vector> // Needed for creating binned output
#include <algorithm>    // std::min_element, std::max_element

/**
 * The `listener` node should *subscribe* to the topic build a histogram of the numbers received (bin-size = 0.1)
* Upon recieving the 50th number the listener should display the histogram somehow and reset its counts
 */

// We can not change the function signature for a Callback to include
// passing other varaiables, therefore the only options would be
// 1. Have a global variable
// 2. Wrap the callback inside a class where this would be a function of a class
// abd we would have a member variable to increment
//
// Though option 2 is preferable, in this example we will show option 1 and
// in the topics masterclass we build upon this and will showcase option 2

static const double bin_size=0.1;
std::vector<double> data;

void chatterCallback(const std_msgs::Float64::ConstPtr& msg)
{
  //ROS_INFO("I heard: [%f]", msg->data);

  if(data.size()<50){
    data.push_back(msg->data);
  }
  else{//Now we have 50, let's create a histogram

    // We use the <algorithm> to find min and max, returns the
    // pointer to the element, if we want the value need to dereference
    double min_val = *std::min_element(data.begin(),data.end());
    double max_val = *std::max_element(data.begin(),data.end());

    // How many bins do we need
    unsigned int hist_size=(unsigned int)((max_val-min_val)/bin_size)+1;
    std::vector<int> histogram(hist_size,0); // Create a vector of size hist_size with all eements as zero
    for(auto elem : data){
      unsigned int idx =(unsigned int) ((elem-min_val)/bin_size);
      histogram.at(idx)++;// increament that particular bin
    }

    // Now let's print to screen stars for every value in bin
    for(unsigned int i=0;i<histogram.size();i++){
      std::cout << i << " ";
      for (unsigned int j=0;j<histogram.at(i);j++){
        std::cout << "*" ;
      }
      std::cout << std::endl;
    }
    data.clear();
    //std::cout << "vector cleared size:" << data.size() << std::endl;

  }

}

int main(int argc, char **argv)
{

  /**
   * The ros::init() function needs to see argc and argv so that it can
   * perform any ROS arguments and name remapping that were provided
   * at the command line. For programmatic remappings you can use a
   * different version of init() which takes remappings directly, but
   * for most command-line programs, passing argc and argv is the easiest
   * way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "listener");

  /**
   * NodeHandle is the main access point to communications with the
   * ROS system. The first NodeHandle constructed will fully initialize
   * this node, and the last NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  /**
   * The subscribe() call is how you tell ROS that you want to receive
   * messages on a given topic.  This invokes a call to the ROS master
   * node, which keeps a registry of who is publishing and who is subscribing.
   * Messages are passed to a callback function, here called chatterCallback.  
   * subscribe() returns a Subscriber object that you must hold on to
   * until you want to unsubscribe. When all copies of the Subscriber
   * object go out of scope, this callback will automatically be
   * unsubscribed from this topic.
   *
   * The second parameter to the subscribe() function is the size of
   * the message queue.  If messages are arriving faster than they are
   * being processed, this is the number of messages that will be
   * buffered up before beginning to throw away the oldest ones.
   */
  ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);

  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this
   * version, all callbacks will be called from within this thread
   * (the main one).  ros::spin() will exit when Ctrl-C is pressed,
   * or the node is shutdown by the master.
   */
  ros::spin();

  return 0;
}
