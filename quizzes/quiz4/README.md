Quiz 4
======

Part A
------

**PREAMBLE**

We have been provided an implementation of the `Controller` and `Quadcopter` classes from Assignment 1. The functionality has been implemented across the base class `Controller` and derived class `Quadcopter`, while there are some errors in the code that prevent it from running correctly. 	

The quiz  has been developed against **Pipes** library ( version 2.5.0 or above ).  We have been provided some tests to let us debug, isolate and fix issues in Quadcopter. The unit tests are in [`utest.cpp` in ./a1/test folder](./a1/test/utest.cpp)` and they can be run from build directory via `./test/utest` and you can also find them in Qtcreator (select `utest to run). There are two tests provided in the `QuadcopterTest` suite, one for TASK 1 and the other for TASK 2.  Before running the unit tests make sure you have run the simulator `roslaunch gazebo_tf uav.launch`. 

**TASK 1 - Initialisation of variables**

All member variables belonging to the class need to be initialised to default values. When we have a base and derived class the initialisation can occur in either the [Base Class Constructor (Controller)](./a1/controller.cpp)  or the [Derived Class Constructor (Quadcopter)](./a1/quadcopter.cpp) .  Initialise the variables in the appropriate constructor so that the `Constructor` tests pass. 

HINT: Check what is being tested in `Contrsuctor` test, look where the variables exist and consider where they should be initialised. Consider that you would also have another derived class `Ackerman`.  

**TASK 2 - Reaching Goal function**

The function  [reachGoal in Quadcopter](./a1/quadcopter.cpp) is not implemented completely. We need to compute the control of total velocity `TARGET_SPEED`, which should be divided into the left/right and forward/backward velocity. To compute how we control the platform we can use the orientation of the platform `odo_.yaw` and the `target_angle_` which tells us where we should be heading. Currently when running 

Compute the correct control value and send the correct command. This will enable completion of `Goals` test which visits two goals.

At present the `reachGoal` will terminate, if we have not reached the goal within 2s of the estimated time. This prevents the code from running infinitely and is a wise move (make a bound by which your code should stop execution).

**TASK 3 - Compute time travelled**

The code should update the total time travelled `time_travelled_` it fails to do this currently. Look at how we compute whether to abort `reachGoal ` as you could use the same function for this task. Also, consider, if we aborted the function `reachGoal`, should we update the time travelled? 

**TASK 4 - Compute distance travelled**

While the estimated distance at the beginning is a rough guide, as the platform moves it could travel more than anticipated (estimated), especially if it  was going against wind (or in case of Ackerman if it was drifting). A better way to computer the distance travelled is increment `distance_travelled_` as you go along.  Also, consider, if we aborted the function `reachGoal`, should we update the distance travelled thus far?  


Part B
-------

The code supplied has two ROS nodes and uses Week09 pre-work. Refer to Week09 module on canvas. We assume you have

* Installed ROS (you can skip this if your using our VM) or have done so for Assignment 1.
* Creating catkin_ws (again, should have done for Assignment 1).
* Compiling our week09 begginer tutorials (adding them to your catkin workspace) - refer Module 09 on canvas.

You will now need to add the quiz4 part a2 once only inside your catkin workspace (my git is ~/git/pfms-2023a-alalemp change for your location of your git ), this is a one of process.

```
cd ~/catkin_ws/src
ln -s ~/git/pfms-2023a-alalemp/quizzes/quiz4/a2 
```

Whenever you need to compile your code you need to be in folder `~/catkin_ws/`and if so, you can compile via `catkin_make` command, and this is done every time you change the code.

To run the two nodes through ROS  (`talker` and `quiz4_a2`) you need roscore running in one terminal via command `roscore` and then can run in another two terminals

```
rosrun beginner_tutorials talker
rosrun quiz4_a2 quiz4_a2_listener
```

To check the unit test you run

```bash
rosrun quiz4_a2 utest
```

We need to implement two functions in  [analysis](./a2/src/analysis.h) 

**TASK 1 - Count characters**

Counts the number of characters in the string supplied (for instance "foo" has 3 characters) 

**TASK 2 - Detect integer in string**

Detects a integer number which is part of the string supplied, detects the first number in the string (for instance "Tommy has 13 cats and 5 dogs" detects 13 as the number.) 

