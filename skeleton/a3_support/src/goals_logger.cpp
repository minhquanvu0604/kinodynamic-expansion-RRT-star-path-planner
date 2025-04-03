#include "ros/ros.h"
#include "geometry_msgs/PointStamped.h"
#include "visualization_msgs/MarkerArray.h"
#include <string>
#include "logger.h"

class GoalsLogger
{
private:
  ros::NodeHandle n_; //!< Node handle for communication
  ros::Subscriber sub_;  //!< Subscriber for goals
  ros::Publisher viz_pub_; //!< Publisher for markers

  std::string filename_; //!< Filename to save goals to
  std::ofstream file_; //!< Handle to file
  visualization_msgs::MarkerArray markerArray_; //!< Marker Array
  unsigned int ct_; //!< Marker Count


public:
  GoalsLogger() : ct_(0)
  {

    ros::NodeHandle pn("~");

    //! By default this code will save goals from this package to GOALS.txt where you run the code <br>
    //! rosrun project_setup goals_logger
    //!
    //! You can also supply a file of goals yourself <br>
    //! rosrun project_setup goals_publisher _goals:=<full filename> <br>
    //! For instance rosrun project_setup goals_publisher _goals:=/home/student/my_goals.txt

    std::string default_filename = "GOALS.txt";


    pn.param<std::string>("goals", filename_, default_filename);
    ROS_INFO_STREAM("file name with goals to be saved:" << filename_);


    sub_ = n_.subscribe("/clicked_point", 1000, &GoalsLogger::goalsCallback,this);
    viz_pub_ = n_.advertise<visualization_msgs::MarkerArray>("visualization_marker",1000);

    file_.open(filename_ , std::ios::out);

    if (!file_.is_open()){
        ROS_FATAL_STREAM ("Can not open " << filename_  << " to save data");
        ros::shutdown();
    }
  }


  ~GoalsLogger(){
      //! Closes file (via handle) if it is open
      if (file_.is_open()){
          file_.close();
      }
  }

private:
  /*! @brief Obtain MarkerArray of CUBES from geometry_msgs::Point
   * The markers are reported in world coordinate frames, namespace goals, type CUBE, colour green
   *
   *  @param goals - vector of geometry_msgs::Point
   *  @return
   */
   visualization_msgs::Marker produceMarker(geometry_msgs::Point pt){


      visualization_msgs::Marker marker;

      //We need to set the frame
      // Set the frame ID and time stamp.
      marker.header.frame_id = "world";
      //single_marker_person.header.stamp = ros::Time();
      marker.header.stamp = ros::Time::now();

      //We set lifetime (it will dissapear in this many seconds)
      marker.lifetime = ros::Duration(1000.0); //zero is forever

      // Set the namespace and id for this marker.  This serves to create a unique ID
      // Any marker sent with the same namespace and id will overwrite the old one
      marker.ns = "goals_clicked"; //This is namespace, markers can be in diofferent namespace
      marker.id = ct_++; // We need to keep incrementing markers to send others ... so THINK, where do you store a vaiable if you need to keep incrementing it

      // The marker type
      marker.type = visualization_msgs::Marker::CUBE;

      // Set the marker action.  Options are ADD and DELETE (we ADD it to the screen)
      marker.action = visualization_msgs::Marker::ADD;

      marker.pose.position.x = pt.x;
      marker.pose.position.y = pt.y;
      marker.pose.position.z = pt.z;


      //Orientation, we are not going to orientate it, for a quaternion it needs 0,0,0,1
      marker.pose.orientation.x = 0.0;
      marker.pose.orientation.y = 0.0;
      marker.pose.orientation.z = 0.0;
      marker.pose.orientation.w = 1.0;


      // Set the scale of the marker -- 1m side
      marker.scale.x = 1.0;
      marker.scale.y = 1.0;
      marker.scale.z = 1.0;

      //Let's send a marker with color (green for reachable, red for now)
      std_msgs::ColorRGBA color;
      color.a=0.5;//a is alpha - transparency 0.5 is 50%;
      color.r=230.0/255.0;
      color.g=230.0/255.0;
      color.b=250.0/255.0;

      marker.color = color;

      return marker;
  }

  void goalsCallback(const geometry_msgs::PointStamped::ConstPtr& msg)
  {

      //! On each callback will save message to file, space seperated x y z values (row for each message)
      geometry_msgs::Point pt = msg->point;
      std::stringstream ss;

      ss << pt.x << " " << pt.y << " " << pt.z << std::endl;

      ROS_INFO_STREAM("Saving: " << ss.str());
      file_ << ss.str();

      visualization_msgs::Marker marker = produceMarker(pt);
      markerArray_.markers.push_back(marker);

      viz_pub_.publish(markerArray_);

  }




};




int main(int argc, char **argv)
{
  ros::init(argc, argv, "goals_logger");

  GoalsLogger GoalsLogger;

  ros::spin();

  return 0;
}
