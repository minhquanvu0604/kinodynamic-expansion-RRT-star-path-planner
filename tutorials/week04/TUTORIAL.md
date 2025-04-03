Week 4 Tutorial Questions
=========================
Work through these questions and make sure you understand what is going on in each example.  If you have any questions about the material, please raise them in the tutorial session.

Today's build is upon solutions in week 03 (ex03) 

Ex01 is pre-work for Ex02-04 we use [shapes folder in starter](./starter/shapes)  

Polymorphism - Ex02
------------------

Building upon your previous solution in the main

* Allow the user to specify number of circles, triangles and rectangles and `max_length`.
* Create the shapes with random lengths to be capped to `max_length` and `setCentre` for each shape to a random values capped at `max_length/2`
* Implement the checkIntercept function for [Isosceles_triangle](https://en.wikipedia.org/wiki/Isosceles_triangle) in `triangle.cpp`
* Allow the user to specify a point to be used for intercept checking `x` and `y`
* Write a function that check if  the shapes intersect the point

Library Creation and Unit Testing - Ex03
-----------------

Building upon your previous solution, we will compile a static library. You will note the  [CMakeLists.txt](./starter/shapes/CMakeLists.txt) is generating a **shared** library called `shapes`. This enables us to unit test the library, which is essential to ensure it performs correctly. We therefore introduce Unit Tests at this point, so you can start to design your own tests for your code.

In [utest.cpp](./starter/shapes/test/utest.cpp) we are checking our implementation of intercept for the Circle. The syntax of the test is `TEST (IntercectTest, Circle)` the `TEST` indicates it is a unit test `IntercectTest` is the suite name and the individual test is `Circle`. So this is a suite of intercept tests and we plan to do this on all shapes. Can you add `TEST (IntercectTest, Circle)` and `TEST (IntercectTest, Triangle)` using the supplied example?

The tests will be compiled at the same time as your code is compiled. You can run all tests with minimal reporting using `make test` or all tests individually from the build directory using `./test/utest`.

Add more tests for the area using a different suite `AreaTest`, you will now need to compare values, and as values are floats the unit test needs to call `ASSERT_NEAR` and example is ` ASSERT_NEAR(a,b,1e-5)` where `a` and `b` are compared to a precision of `1e-5`.

There is a very comprehensive guide called [Googletest Primer](https://github.com/google/googletest/blob/master/docs/primer.md) .



ADVANCED MATERIAL 
-----------------

The below material is classified advanced and will not be examinable, it is here for completeness should you desire to create libraries

### Library Installation - Ex04

We can now install the library to be part of our system. 

If you have `sudo` access on your computer, the install process will install your library to default install locations which on a Linux system is `/usr/local`

Alternative, you can specify to be any directory of choice and we have two examples below that can be uncommented
* 1. Installs to your build directory `set(CMAKE_INSTALL_PREFIX ${CMAKE_BINARY_DIR})`
* 2. This would install to your home folder and within subfolder of project_name (ie /home/user/shape_library) `set(CMAKE_INSTALL_PREFIX "$ENV{HOME}/${PROJECT_NAME}")`


To compile the executable
```bash
mkdir build
cd build
cmake ..
make
make install
```
Keep note where the install directory is (if it is not in the system)

### Library Linking - Ex05

Create a project using the `shape_processing` folder and  [CMakeLists.txt](./starter/shape_processing/CMakeLists.txt) that links to your library static.

Modify the `CMakeLists.txt` and add the `include_directories` and `link_directories` if needed (if your library is not installed to be part of system).

Modify the main (which uses the library) and

* Allow the user to specify the number of circles and rectangles
* Create the shapes with random lengths to be capped to `MAX_LENGTH` - which is a const in [shape_processing.h](./starter/shape_processing/shape_processing.h).
* Main allows user to enter a location x,y within (`-MAX_SIZE`, `MAX_SIZE`). `MAX_SIZE` is a const in [shape_processing.h](./starter/shape_processing/shape_processing.h)


Create a processing class [ShapeProcessing](./starter/shape_processing/shape_processing.cpp)

* Constructor accepts `Shape*`
* On each iteration, the intersect is checked against all the shapes in `checkAllIntersects` and shapes that are intersected are removed.

