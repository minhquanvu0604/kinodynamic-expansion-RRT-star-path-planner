#include <iostream>

int main () {

    double x = 0.0;
    x = 41012;
    std::cout << "x = " << x << std::endl;

    double* ip = &x;
    std::cout << "value at ip = " << *ip << std::endl;
    std::cout << "value of ip = " << ip << std::endl;

    //    * Make variable y a reference to x (what type is y?)
    double &y = x;

    //    * Print the value of what y is referencing to
    std::cout << "y = " << y << std::endl;

    //    * Create a double z of value 1
    double z = 1.0;
    std::cout << "z = " << z << std::endl;

    //    * Use pointer ip to point to z
    ip = &z;
    std::cout << "value at ip = " << *ip << std::endl;

    //    * Make y a reference to z (is this possible?)
    // NOT POSSIBLE, can't change the reference of y, just value

    //    * Assign y the value of z
    //    * Assign z the value 100
    y = z; // Only changing the value not the reference
    z = 100;

    //    * Print the value of what ip is pointing to
    //    * Print the value of x
    //    * Print the value of y
    std::cout << "value at ip = " << *ip << std::endl;
    std::cout << "x = " << x << std::endl;
    std::cout << "y = " << y << std::endl;

    return 0;
}
