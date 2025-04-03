Week 1 Tutorial Questions
=========================

We examine
* functions
* structs
* vectors


Ex01 - Functions 
---------
* Create a function that accepts a double value as a parameter and
* Returns a bool value if the double is greater than zero and the square value instead of initial passed value.

Question:
* What is a function declaration (also referred to as functions signature)?

Ex02 - Functions 
---------
* Create an additional function that accepts a double `value` as a parameter and
* Returns the following
  * bool indicating if `value`is greater than zero
  * square of `value` instead of the initially passed `value`
  * passed `value` incremented by one


Questions:
* Given we have three things to return from a function, how do we achieve this? 
* Could we call this function the same name as the existing function created in Ex01?

Ex03 - Struct
------
* Create a structure called `Sensor` than contains
  * A variable for the number of samples `num_samples`
  * An array of samples `double data[]`

* Create a variable called `sensor` of type `Sensor`
* Populate data of `sensor` with 5 elements, each data[i] =i. How to code end of loop?
* Create a function that prints samples 
* Can you initialise a sensor of two elements in one line of code?

Questions:
* What is the problem with using an array here? Why do we need num_samples?

* What happens if we exceed the size of array (by assigning to data[i] where i is > then size of array)?

Ex04 - Vector
------
* Create a vector of integers called  `data` of length 10 elements 
* Create a loop to populate elements of `data` (each element at location i = value i))
* Create a function to loop over elements of vector `data` and print to screen


Questions:
* How to create a vector of floats?
* How could we pass the length to the executable instead of hard coding 10?

Ex05 - State Machine : Bring concepts together
------
We are required to implement a controller for a air condition system, and we have been given a UML State-chart Diagram. The image below is from an extract on State Machines ([Lecture Notes on Computer Science](https://praveenthomasln.wordpress.com/2012/04/07/state-machines-s8-cs/)) and an [explanation of the syntax here](https://www.lucidchart.com/pages/uml-state-machine-diagram).



![](https://praveenthomasln.files.wordpress.com/2012/04/figure-1-state-machines.png)



Questions:

* How many states does our air-con have? 

* Does any state have sub-states, if so which state has sub-states and what are they?

* What is the input to the system?

* What needs to be checked to transition between states?

  

To implement a simulator of the air-con we will simplify it (as we don't have a sensor or button to User Interface - UI) and therefore:

* Desired temperature to be provided on startup (via std::cin) 1-50C

* Current temperature to be a random number between 10-50C

* We immediately enter IDLE

* We enter shutDown (terminate program) if in IDLE state and have been running for 3 minutes

* We run our program in a loop (every 5s) which either increments or decrements the temperature by 0.5C

Revisit [prework](./PREWORK.md) to see enums, which are quite handy for states! Use the enums for states, created a switch/case section of code using them. 

You will need

* Accepting user input
* functions to increment/decrement temperature (heat/cool) - a function allows more flexibility
* function to [determine elapsed time](https://www.cplusplus.com/reference/chrono/steady_clock/now/) and slow down your system via [sleep_for](https://en.cppreference.com/w/cpp/thread/sleep_for)
