Week 1 Pre-work
=========================

Please complete refresher of pointers, arrays and enums.

Ex01 - Pointers & Reference
--------------------

Work with [pointers.cpp](./starter/pointers.cpp)

* Assign to variable x (of type double) the value 41012
* Use a pointer ip to point to x (why type is ip?)
* Print the value of what ip is pointing to
* Print the value of ip
* Make variable y a reference to x (what type is y?)
* Print the value of what y is referencing to
* Create a variable z (of type double) and assign it value 1
* Use pointer ip to point to z (instead of x)
* Assign y the value of z
* Assign z the value 100
* Print the value of what ip is pointing to
* Print the value of x and y 

Ex02 - C Arrays
--------

Work with [arrays.cpp](./starter/arrays.cpp)

* Create an array x of doubles with 10 elements
* Initialise the elements of the array on creation, each element [i] has value i
* Alternatively to above, create a loop to populate elements of x (each x[i] =i), how to code end of loop?)
* Can you use a pointer and loop to initialise elements of array?

Ex03 - Enums + switch/case statements
--------

Enums ([enumeration](https://en.cppreference.com/w/cpp/language/enum)) allows us to set a distinct type whose value is restricted to a range of values. This is useful when we know the permissible set of values, allows us to define them by name and refer to them for readability of our code. 

Work with [enum_example.cpp](./starter/enum_exammple.cpp)

In the example you will find two enums, one for `color` that has a range of values `red,green` and another for `altitude`, which we specify to be a char of either `high, low`   

TASK

* Add two more values to color `yellow (value 1) and blue (value 21)`. 
* Add the values to the switch statement in function `std::ostream& operator<<(std::ostream& os, color c)`
* Add a default case in switch statement `std::ostream& operator<<(std::ostream& os, color c)`

We here show the power of C++ and the `operator<<` . If your coming from C and have use printf, shelve that. C++ allows us to use the insertion (`<<`) operator to write [standard data types](https://en.cppreference.com/w/cpp/language/types) to terminal or to a file using `<<` (here we write to std::cout  - standard out to terminal). If we have custom types (such as enums or structs) then we need to create our own `operator<<` for them, but can then in the future simply print a variable using `<<`.

