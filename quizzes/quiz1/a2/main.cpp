#include <iostream>
#include "rectangle.h"


int main () {
    //! TODO: Create a rectangle
    Rectangle rectangle1;
    rectangle1.setHeightWidth(5.0, 3.55);

    // Print some info about it
    std::cout << "The area of " << rectangle1.getDescription() << " is " << rectangle1.getArea() << std::endl;
    std::cout << "It is a " << rectangle1.getDescription() << std::endl;

}


