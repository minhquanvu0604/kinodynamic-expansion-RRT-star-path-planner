// Includes std::cout and friends so we can output to console
#include <iostream>

// Ex01. Returns a bool value if the double is greater than zero
// and the square value instead of initial passed value
bool squareOfCheckPositive(double &value) {
    bool is_positive = value > 0.0;
    value *= value;
    return is_positive;
}

// Ex02. Returns bool value if the double is greater than zero, the square value, the cube value and the passed value incremented by one
bool squareCubeIncrement(double &value, double &increment) {
    double square = value*value;
    increment = value + 1;
    value=square;
    return value > 0.0;
}


// ADVANCED: How best protect the passed value?

// Every executable needs a main function which returns an int
int main () {

    double x = 2.0;


    // Ex01.
    double y = x;
    std::cout << x << " is ";
    bool result = squareOfCheckPositive(y);
    if (result) {
        std::cout << "positive ";
    } else {
        std::cout << "not positive ";
    }
    std::cout << "and its square is " << y << std::endl;

    // Ex02.
    double y1=y;
    result =  squareCubeIncrement(y, y1);
    if (result) {
        std::cout << "positive ";
    } else {
        std::cout << "not positive ";
    }
    std::cout << "and its square is " << y << std::endl;
    std::cout << "and increment is " << y1 << std::endl;

    return 0;
}




