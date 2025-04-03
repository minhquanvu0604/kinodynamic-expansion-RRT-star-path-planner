Week 10 Tutorial Questions
=========================

Symbolically link the `topics_masterclass` package supplied in week10 starter packages:
```bash
cd ~/catkin_ws/src 
ln –s <your_git_repo>/tutorials/week10/wk10_starter .
```
Build the package using the `catkin_make` command

## Working with Laser 

Run the core using `roscore` in a terminal  and run the simulation
`rosrun stage_ros stageros /opt/ros/$ROS_DISTRO/share/stage/worlds/simple.world`

We will be modifying the `week10_laser` package in `topics_masterclass` folder and  can run our code we have been developing using `rosrun week10_laser sample` 

### TASK1: Find the closest point [x,y] to the robot in local coordinates (relative to robot) using sensor_msgs::LaserScan
See the `laserCallback` function of the `PfmsSample` class to complete this exercise.

First we need to select robot_0 and identify what data is from laser scanner. We need to modify the code so we can subscribe to the laser data from this platform. Then, implement your code that finds the closest point in `LaserProcessing::closestPoint()`. You will find that originally, this function is called from the callback `PfmsSample::laserCallback`

 **Your task is to print the location of the nearest obstacle in the laser scan. The location should be given as the x, y position of the obstacle *relative* to the robot's coordinate frame.**

* On command line type 'rosmsg show sensor_msgs/LaserScan'
* What are we provided in this message?
* Do we have the information in this message to find the closest point?
* What part of the message do we need to iterate over?
* How do we convert from range data to {x,y} [this is known as polar to cartesian](https://www.mathsisfun.com/polar-cartesian-coordinates.html)
* Where is the time of this message stored?
* Is the closest point identified the same as the one you see as closest on the stage simulator? Why is this the case?

### TASK 2: Point in Global Coordinates
We will now look at transfering the data into "world" coordinates. We need to take the position of the robot into consideration and attempt to publish the data in "world" refernce frame.

As we want to utilise both sources of data (robot position and the laser that is on the robot), we have `PfmsSample::seperateThread()` which we will use to combine both sources of data and then compute the position in global coordinates.

We will use the  [MarkerArray](http://docs.ros.org/melodic/api/visualization_msgs/html/msg/MarkerArray.html) to show items on rviz, it allows drawing arrows, axis, cylinders, squares, lines etc. 

Find the closest point in global coordinates {x,y} and publish it in MarkerArray, refer to the code for an example.

### TASK 2B: Changing Topic Names

Can you run your code for `robot_1`? What needs to change and can it be done on the command line rather  that changing your source code.



## Working with Controlling a Platform

We will use `week10_quad` package in `quadcopter_control` folder and use `rosrun week10_quad sample` to run our code. 

To run the simulator `roslaunch gazebo_tf uav_a3.launch gui:=true` if it complains about the launch file, complete a `git pull` from the `pfms-support` package.

TASK 3: Controlling the Quadcopter
-----------------------------------------------
We need to subscribe and publish to topics to control the quadcopter. 

Consider:

* which topics do we need to send controls to the quadcopter? What are the topic names and data types?
* which topics do we need to subscribe to, in order to determine where we are? 
* how do we give the quadcopter a goal to go to? Can we do this via a topic?

We will need to modify the Quadcopter/Controller class (member variables) as well as the constructor(s) and `Quadcopter::sendCmd` function as well as `Controller::getOdometry` 


TASK4: Subscribe to another topic
---------------------------

This is a strech goal for today, let's receive information directly from rviz into this node, receive a [ClickedPoint from RViz](https://answers.ros.org/question/69019/how-to-point-and-click-on-rviz-map-and-output-the-position/) into this node

* Find the topic name `/clicked_point` in the list of topics and determine its type
* Create a callback for this topic name and correct type
* Print out the sequence number and x , y coordinates using `ROS_INFO_STREAM`


[ROS Installation Instructions]: http://wiki.ros.org/ROS/Installation
[ROS Tutorials]: http://wiki.ros.org/ROS/Tutorials
[ROS TF]: http://docs.ros.org/diamondback/api/tf/html/c++/namespacetf.html
[Polar to Cartesian]: https://www.mathsisfun.com/polar-cartesian-coordinates.html
