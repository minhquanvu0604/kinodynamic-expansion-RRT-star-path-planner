Week 9 Tutorial Questions
=========================

ROS has a central server called `roscore` that monitors all nodes (executables in ROS), and connects topics and services. You will need to have a `roscore` running. A good practice is to run it in its own terminal. If your launching a number of nodes ROS via `roslaunch` then ROS will start the core if one is not present.

Start roscore in one terminal
```bash
roscore
```


Ex01: Modifying a ROS Node
---------------------------

Modify the provided package to do the following:

* Change the message type to publish/subscribe to a double.
* Every 0.2 seconds the `talker` node should draw samples from a Gaussian distribution (mean = 0.0, std dev = 1.0) and *publish* the samples to a topic
* The `listener` node should *subscribe* to the topic build a histogram of the numbers received (bin-size = 0.1)
* Upon receiving the 50th number, the listener should display the histogram somehow and reset its counts 

### Questions ###

* Do you need to change the Message? refer to http://wiki.ros.org/std_msgs
* Will you use a container for the histogram?
* How would you display it?

Ex02: Stage as a Simulator
----------------------------

If you do not have it already, you will also need the `stage_ros` package, install it as follow 
```bash
sudo apt-get install ros-$ROS_DISTRO-stage-ros
```
Start stage_ros in another terminal 
```bash
rosrun stage_ros stageros /opt/ros/$ROS_DISTRO/share/stage/worlds/simple.world
```
You should have the simulator appear
![Simple World in Stage Simulator](http://4.bp.blogspot.com/_B6REL4AVpFA/Szk9ipweWTI/AAAAAAAAALc/orflaXzpcZk/s400/Picture+2.png)

The simulator has a robot equipped with a Laser and a Sonar Array (number of Sonars). The Sonar Array, unfortunately is not supported in ROS, so a single sonar reading is reported.

We have now used `rostopic` and `rosmsg` in particular `rostopic info <topic_name>` and `rostopic echo <topic_name>` and also `rosmsg info <msg_name>`. In the next execrise you will use them extensively.

### Questions And Tasks ###

* What are the topics available?
* Name two differences between the `/robot_0/cmd_vel` and  `/robot_0/base_scan_1` topics?
* What are the differences between `/robot_0/base_scan_0`  `/robot_0/base_scan_1` topics?
* Print the contents on the `/robot_0/base_scan_1` topic to screen using `rostopic echo <topic_name>`
* What are the fields in the message?
* Print just the `ranges` values from `/robot_0/base_scan_1` topic
* Print just the first reading from `ranges` values from `/robot_0/base_scan_1` topic (HINT: You have a vector, how do you access the first element of it?)
* What message type is being sent on  `/robot_0/cmd_vel`? (HINT: use `rostopic` info and `rosmsgs info`
* Send a linear velocity of 0.1 m/s to the `/robot_0/cmd_vel` every `0.1s` topic using `rostopic pub` (HINT `rostopic pub -h` can tell you how to send data at a specific rate)


[ROS Installation Instructions]: http://wiki.ros.org/ROS/Installation
[ROS Tutorials]: http://wiki.ros.org/ROS/Tutorials
[ROS TF]: http://docs.ros.org/diamondback/api/tf/html/c++/namespacetf.html
[Polar to Cartesian]: https://www.mathsisfun.com/polar-cartesian-coordinates.html

