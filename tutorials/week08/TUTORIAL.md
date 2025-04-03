Week 8 Tutorial 
=========================
Work through these questions and make sure you understand what is going on in each example. If you have any questions about the material, please raise them in the next tutorial session.

In order to run the unit tests, they have been developed against **Pipes** library ( version 2.5.0), same library as your assignment 2. 


-------

Unit Testing 
===============================

We will use Google's C++ unit testing framework. A good starting point for using googletest is to read the [primer] document. 

Our examples today revolve around making Unit Tests for an example of Assignment 1, we are testing the functionality of code according to specifications. We will be designing tests that test the code as per expected output. 

We will be using a library built by one of your colleagues for Assignment 1 and testing it with the specifications at hand. You have been provided the compiled libraries [`example` in ./starter/ex01/dep/lib](./starter/ex01/dep/lib) and the header files that allow us to use the library [ in ./starter/ex01/dep/include](./starter/ex01/dep/include).  

To compile the unit tests from within folder `starter/ex01`

```bash
mkdir build
cd build
cmake ..
make
```

Task 1
-------

The [test_constructors.cpp](./starter/ex01/test/test_constructors.cpp) needs to  check variables have been initialised correctly on creation of `Ackerman` class. 

What functions should we check? Can we guarantee the use of the class will call any function of the class in a particular order? What should be initialised to zero when Ackerman is created?

Prior to running the unit test run the simulator `roslaunch gazebo_tf multi.launch` in a terminal window. Then run the unit test from the `build` directory via `./test/constructorsTests`

**TASK**: Implement the constructor tests  for `Ackerman`

Task 2
-------

Examine the [test_audi.cpp](./starter/ex01/test/test_audi.cpp). This unit test puts under scrutiny the `Ackerman` class calculation of `distance` and `time` to target. Unlike Task 1, here the unit tests are part of `TEST_F` `AckermanTest` group. 

The difference with `TEST_F` is that they allow to setup variables that are persistent between all tests via `SetUpTestCase` and `TearDownTestCase`. This is slightly beyond scope of the subject, though if interested examine the [ackerman_test.h](./starter/ex01/test/ackerman_test.h).

The unit tests examine the public interfaces of the `Ackerman` class and we have 2 unit tests that we have developed for two goals.

Prior to running the unit test run the simulator `roslaunch gazebo_tf multi.launch` in a terminal window (if it was already running, terminate it and restart). Then run the unit test from the `build` directory via `./test/audiTests`

**TASKS** to complete are:

- [ ] A) Create a unit tests that fails as `Ackerman` can not reach a point (reachable should be false)
- [ ] B) Test `Ackerman` going in a straight line, often division by zero is not handled well so it's worth testing.
- [ ] C) Create a unit tests for another location for which you have computer the distance and time from a seperate source.

Task 3
-------

Examine the [test_audi_reach_goals.cpp](./starter/ex01/test/test_audi_reach_goals.cpp). This unit test puts under scrutiny the `Ackerman` class calculation of `distance` and `time` to target. Like Task 1, here the unit tests are part of `TEST` `AckermanExTest` group. 

The unit tests examine the public interfaces of the `Ackerman` class in reaching goals, we have 1 unit test that we have developed to reach a goal.

Prior to running the unit test run the simulator `roslaunch gazebo_tf multi.launch` in a terminal window (if it was already running, terminate it and restart). Then run the unit test from the `build` directory via `./test/audiExGoals`

**TASK** to complete: Create Another test called `2ndGoal` in same `AckermanExTest` group

From the current position (upon completion of 1stGoal) reach a goal with `x=9 y=8`  tolerance of `0.5` to reach it, the anticipated distance shoudl be `19.1748` and time `6.5893`  .

**Outside of class, contemplate how you could test your Assignment 2**    


[primer]: https://github.com/google/googletest/blob/master/docs/primer.md
