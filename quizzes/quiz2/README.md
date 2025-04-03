Quiz 2
======

In both quizzes DO NOT CHANGE ANY OF THE EXISTING function declarations. You can add other private functions, but your code will only be tested against the currently defined public functions.

Part A
------

**Preamble**

We are in charge of a drag race and will design a class `Analysis` to handle this. The class has two constructors (one of which takes a visualiser). 

You will need to utilise the car library which is provided for the Task (you don't need to write it), refer to classes [CarInterface](./a1/dep/include/carinterface.h) which is the interface class and examine [Car](./a1/dep/include/car.h). You will need to pass the cars via `CarInterface*` to the [Analysis](./a1/analysis.h) class for further processing, look at the constructor of `Analysis`. 

We have a `demoRace` that is just a small demo to show how to accelerate and decelerate the cars.  You can comment this out in your code, once you have a feel for how to control the cars. 
You have to keep accelerating to go faster, and decelerate to slow down (functions available), refer to class [Car](./a1/dep/include/car.h) . You will need to create some cars to run the demo race (determine how to create an object of `Car`), the demo shows the car on a circular track as they race. Unit tests are provided for Task 2 and Task 3.
You can modify the main to create three vehicles with provided specifications for your own visualisation. You will need to create some cars to run the demo race (determine how to create an object of `Car`). We have a `demoRace` that is just a small demo implemented in `Analysis` to show how to accelerate and decelerate the cars and shows the car on a circular track as they race.  You can comment calling this function in your main, once you have a feel for how to control the cars. You have to keep accelerating to go faster, and decelerate to slow down (functions available), refer to class [Car](./a1/dep/include/car.h) . The cars only accelerate to a top speed that depends on the car parameters, and decelerate to zero. 

**TASK 1- Sorting cars by odometry**

Implement the `sortByOdometry` sorting algorithm, that will sort the vehicles per kilometres in their odometry, sorted in ascending [label](order). You can get some inspiration in completing this using pairs (the value to be sorted as first element, and the previous index as second element) from [sorting pairs](https://www.geeksforgeeks.org/keep-track-of-previous-indexes-after-sorting-a-vector-in-c-stl/).

Example below, the vector returned has value {10,22,7} as this orders the car per odometry in ascending order.

| Car ID | Odometry | 
| ------ | -------- | 
| 10     | 12788.9  | 
| 7      | 68833.1  | 
| 22     | 56686    | 

**TASK 2 - Standard drag race **

Create a drag racing algorithm, where each vehicle will race a distance of 100m and then stop. So the logic for `dragRace` is:

  1. Accelerating each car 
  2. When all cars cover 100m the race is finished, we declare the winners as the first car to cover (travel) 100m. 

 An example below, the vector returned has value {4,42,21} in order of crossing the 100m line

| Car ID | Place                   |
| ------ | ----------------------- |
| 4      | 1st to cover 100m        |
| 21     | 3rd (last) to cover 100m |
| 42     | 2nd to cover 100m        |

HINTS: (1) use a loop to do this by going over a container of vehicles (2) what do we need to query at beginning of race (3) what function needs to be called during race 

**TASK 3 - Stop all cars  **

Though we did a drag race, we didn't stop the vehicles (decelerate them to zero speed). Create a function `stopAllCars` in the `Analysis` class to achieve this. This function needs to stop (bring speed to zero) for all cars.

HINTS: (1) use a loop to do this by going over a container of vehicles, (2) what other information needs to be stored to know if to further decelerate each vehicle, (3) remember dangers of comparing float numbers to zero?

**TASK 4 -  Zero - Top Speed - Zero race**

Initial drag race (in TASK 2) favoured high horse power, we have designed another race that is more about power to weight ratios and breaking force. All cars have to reach their individual top speed and then go back to zero speed.  So the logic for `zeroTopZeroRace` is

 1. Accelerate all cars
 2. As each individual vehicle reaches top speed decelerate that car, keep other cars that have not reached their top speed accelerating
 3. When all vehicles have stopped, terminate the race

 Return the order of vehicles that reached zero speed (the car MUST have reached top speed before coming back to zero speed)

Example below, , the vector returned has value {1,43,16} 

| Car ID |            |
| ------ | ---------- |
| 16     | 3rd (last) |
| 1      | 1st        |
| 43     | 2nd        |



HINTS: (1) use a loop to do this by going over a container of vehicles (2) what function(s) needs to be called during race (3) what other information needs to be stored to know when to decelerate each vehicle and where would you store this


Part B
------

1. TASK: Implement function `void populateContainer(std::deque<double>& container, unsigned int num_values, double element)` that accepts a container and modifies it by adding user specified numbers of elements to the front of container. The actual element is also supplied by user (for instance num_values =4 element =-1.5) ; this would result in four elements of -1.5 are added to beginning of deque)  [container_ops.cpp](./a/container_ops.cpp)

2. TASK: Implement function t`void bubbleSortContainer( std::deque<double>& container)` that accepts a deque container and rearranges elements by bubble sort operation.

   An example of C++ code for arrays is here https://www.programiz.com/dsa/bubble-sort . 

