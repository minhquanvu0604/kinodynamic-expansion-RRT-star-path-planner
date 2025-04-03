Week 2 Tutorial Questions
=========================
Work through these questions and make sure you understand what is going on in each example. If you have any questions about the material, please raise them in the next tutorial session.

Classes - Ex01
--------------------
Implement a circle class that has:
* A constructor that sets the radius
* A Method that sets the radius
* A Method that returns the area
* A Method that returns the perimeter
* A Method that sets the area
* A Method that sets the perimeter

Questions:
* What data should we store in the class
* How do we set the accessibility of methods (member functions) and member variables?

Classes - Ex02
--------------------
Write a program that 
* Creates three circles (or radius 1.0, 2.0 and 5.0)
* Computes the area of these circles
* Computes the perimeter of these circles

Questions:
* Is there a way we can store multiple objects of the same class in a container?

Classes - Ex03
------------------
Write a: 
* Function that accepts a vector of circles and returns the combined area of all circles in the vector.
* Program that creates a vector of circles with random radii (the radii can be between 1.0 and 10.0) and uses the function to compute the combined area.

Hints:
[Random Generator](http://www.cplusplus.com/reference/random/uniform_real_distribution/)
* Use a seed for a Random Number Generator 
* Allow user to specify how many random circles are generated

Questions:
* If we used a C style array, what would be the pinch points here? How could we ensure the program does not cause a segmentation fault?
* What method in STL allows access to elements of the container?
* What is a segmentation fault?
* What is an exception?

Bonus Questions:
* If we wanted to create a circle of random size within Circle class, how should we use the Random Number Generator?

Air Conditioner Class - Ex04
------
We repeat week 01 example of Air-Condition conreoller, but this time we implement a class for a air condition system. This allows us to have a default state, and hide a number of function implementations from the user of the class.

We have been given a UML Statechart Diagram of the controller. The image below is from an extract on State Machines ([Lecture Notes on Computer Science](https://praveenthomasln.wordpress.com/2012/04/07/state-machines-s8-cs/)) and an [explanation of the syntax here](https://www.lucidchart.com/pages/uml-state-machine-diagram).



![](https://praveenthomasln.files.wordpress.com/2012/04/figure-1-state-machines.png)



To implement a simulator of the air-con we will simplify it (as we don't have a sensor or button to User Interface - UI) and therefore:

* Desired temperature to be provided on startup (via std::cin) 1-50C
* Starting current temperature to be a random number between 10-50C
* We immediately enter IDLE
* We enter shutDown (teminate program) if in IDLE and we have been running for 3 minutes
* We run our program in a loop (every 5s) which either increments or decrements the temperature by 0.5C

HINTS:
* function to [determine elapsed time](https://www.cplusplus.com/reference/chrono/steady_clock/now/) and slow down your system via [sleep_for](https://en.cppreference.com/w/cpp/thread/sleep_for)
