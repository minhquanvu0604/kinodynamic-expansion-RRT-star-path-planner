#include "ros/ros.h"
#include <ros/package.h> //This tool allows to identify the path of the package on your system
#include "geometry_msgs/PoseArray.h"
#include "visualization_msgs/MarkerArray.h"
#include <string>
#include "logger.h"
#include <thread>


/*! @brief Obtain MarkerArray of CUBES from geometry_msgs::Point
 * The markers are reported in world coordinate frames, namespace goals, type CUBE, colour green
 *
 *  @param goals - vector of geometry_msgs::Point
 *  @return
 */
visualization_msgs::MarkerArray produceMarkerList(std::vector<geometry_msgs::Point> goals){

    visualization_msgs::MarkerArray markerArray;
    unsigned int ct=0;

    for (auto pt:goals){
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
        marker.ns = "goals"; //This is namespace, markers can be in diofferent namespace
        marker.id = ct++; // We need to keep incrementing markers to send others ... so THINK, where do you store a vaiable if you need to keep incrementing it

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
        color.r=0;
        color.g=1.0;
        color.b=0;

        marker.color = color;

        markerArray.markers.push_back(marker);
    }
    return markerArray;
}



int main(int argc, char **argv)
{

    ros::init(argc, argv, "goals_publisher");

    ros::NodeHandle n;

    //! By default this code will load goals from this package (data/goals.txt)
    //! rosrun project_setup goals_publisher
    //!
    //! You can also supply a file of goals yourself
    //! rosrun project_setup goals_publisher _goals:=<full filename>
    //! For instance rosrun project_setup goals_publisher _goals:=/home/student/my_goals.txt

    //! Below allows to find the folder belonging to a package, if you change the package where you store data you need to update the name here
    std::string path = ros::package::getPath("a3_support");
    path += "/data/"; //Looking at data subfolder
    std::string default_filename = path + "GOALS.TXT";

    std::string filename;
    ros::NodeHandle pn("~"); // We need a private node handle for this
    pn.param<std::string>("goals", filename, default_filename);
    ROS_INFO_STREAM("file name with goals to be loaded:" << filename);

    std::vector<geometry_msgs::Point> goals;

    bool OK = logger::loadPoints(filename, goals);

    if(!OK){
        ROS_WARN_STREAM("Unable to read the goals from :" << filename);
        return -1;
    }

    geometry_msgs::PoseArray msg; // This is our message type
    msg.header.seq =0;
    msg.header.frame_id = "world";
    msg.header.stamp = ros::Time::now();

    for(auto goal:goals){
        geometry_msgs::Pose p;
        p.position = goal;
        p.orientation.w=1.0; // Just making the quaternion proper (sums to 1)
        msg.poses.push_back(p);
    }

    //Publish goals
    ros::Publisher goal_pub = n.advertise<geometry_msgs::PoseArray>("goals", 1000,true);
    //Publishing markers
    ros::Publisher viz_pub = n.advertise<visualization_msgs::MarkerArray>("visualization_marker",1000);

    std::this_thread::sleep_for(std::chrono::milliseconds(200)); //Wait to allow connection to be established


    visualization_msgs::MarkerArray marker_msg = produceMarkerList(goals);


    ROS_INFO_STREAM("We are sending " <<  msg.poses.size());

    goal_pub.publish(msg);
    ros::spinOnce();
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    ROS_INFO("We have sent goals");

    viz_pub.publish(marker_msg);
    ros::spinOnce();
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    return 0;
}
