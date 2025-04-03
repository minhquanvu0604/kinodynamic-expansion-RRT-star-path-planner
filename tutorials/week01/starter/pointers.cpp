#include <iostream>

int main () {

// Refer to README.md for the tasks required, we adress first 3 tasks.

//    * Assign to variable x (of type double) the value 41012
    double x = 41012;
    std::cout << "x = " << x << std::endl;

//    * Use a pointer ip to point to x (why type is ip?)
    double* ip = &x;

//    * Print the value of what ip is pointing to
    std::cout << "value at ip = " << *ip << std::endl;

//    * Print the value of ip
    std::cout << "value of ip = " << ip << std::endl;

//! Remaining tasks here are for students to complete

//    * Make variable y a reference to x (what type is y?)
    double& y = x;

//    * Print the value of what y is referencing to
    std::cout << y;

//    * Create a double z of value 1
    double z = 1;

//    * Use pointer ip to point to z
    ip = &z;

//    * Make y a reference to z (is this possible?)
    y = z;

//    * Assign y the value of z
    y = z;

//    * Assign z the value 100
    z = 100;

//    * Print the value of what ip is pointing to
    std::cout << *ip << std::endl;

//    * Print the value of x
    std::cout << x << std::endl;

//    * Print the value of y
    std::cout << y << std::endl;


    return 0;
}
