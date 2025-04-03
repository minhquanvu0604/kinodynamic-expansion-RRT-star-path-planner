Week 7 Tutorial Questions
=========================
Work through these questions and make sure you understand what is going on in each example. If you have any questions about the material, please raise them in the tutorial session.

Ex01 - Convars
--------------------

Consider the [ex01 starter code](./starter/ex01) example, which uses two threads to do some work in parallel, one generates data, the otehr computes a fibonacci sequence sum based on this number (so this could take a long amount of time). 
Questions to reflect on:
* What are the data structures used between threads, what role do the threads have? 
* What is CPU usage of this process? (hint, use top / htop)
* What value would you expect to see printed? 

Compile and run the code

TASK:
* Change operating directly on mutex with `lock` method to using a `unique_lock`

* Guarantee the `processSamples` runs immediately after new data is generated. Use a [conditional variable](https://en.cppreference.com/w/cpp/thread/condition_variable).  
The `convar` uses mechanism of `waiting` and `notifying`. Here we want one thread (the one that produces data) to notify the other when data is available. The other thing to consider is the `predicate` in `cv.wait(lk, []{return ready;});` the `return ready` here is evaluated when the thread checks should be released from the wait. What would we be checking?

Ex02 - Convars
--------------------

We embed the `data`   `mutex` and `convar` in a `struct` called `Data` within namespace `pfms`. The use of namespace is recommended by the C+ coding standard we are following. The class `Sensor` contains `generateData` functionality and `Analysis` class has`processSamples` . The main connects the two classes by passing the struct `data` to both classes.  

Compile and run the code

TASK:

* Change operating directly on mutex with `lock` method to using a `unique_lock`

* Guarantee the `processSamples` runs immediately after new data is generated. Use a [conditional variable](https://en.cppreference.com/w/cpp/thread/condition_variable).  
  The `convar` uses mechanism of `waiting` and `notifying`. Here we want one thread (the one that produces data) to notify the other when data is available. The other thing to consider is the `predicate` in `cv.wait(lk, []{return ready;});` the `return ready` here is evaluated when the thread checks should be released from the wait. What would we be checking?

Ex03 - Threading Example
--------------------

We have been provided with the source code of a sensor (radar), with a blocking call function [getData](./starter/ex02/radar.h), thus rendering the main blocked until data is available.  We are required to find the closest target from 3 radars.

Complete the following:
* Change the Radar class, so the thread of execution is inside the radar via the [start](./starter/ex02/radar.h) member function (the class will run now with a separate thread generating data
* Create 3 instances of the radar class and run the corresponding threads

Now create the Data Processing class that would find the closest target
* What do we need to pass to this class

Implement the following
* Find the closest target from all 3 radars every 50ms. How could we guarantee this occurs?
* Find the closest target from a radar whenever that sensor provides new data

