// We need to include the declaration of our new circle class in order to use it.
#include "circle.h"

#include <iostream>
#include <vector> // This header is needed to utilise vectors

// Defining functions like this above main is not best practice
// We will talk later about where this function belongs
double computeTotalArea(std::vector<Circle> circles) {
    double total_area = 0;
    for (auto circle : circles) {
        total_area += circle.getArea();
    }

//    //An alternative would be to iterate for all elements in vector
//    for (unsigned int i=0; i<cirlces.size(); i++) {
//        total_area += circles.at(i).getArea();
//    }

    return total_area;
}

int main () {

    // Create an empty vector of circles
    std::vector<Circle> circles;


    //1) Create three circles (or radius 1.0, 2.0 and 5.0)
    //
    // To demonstrate adding an object of class circle to a vector we show three seperate ways of doing this.


    //First way to create an object and add it to vector could be achieved below
    Circle circle(1.0);
    circles.push_back(circle);

    //If we did not need the object, rather create the circle and add to vector in one step
    circles.push_back(Circle(2.0));


    //Finally, if we wanted to use an iterator to add the vector to a specific location
    //http://www.cplusplus.com/reference/vector/vector/emplace/
    circles.emplace(circles.end(),Circle(5.0));


    //2) Compute the area of these circles
    double total_area = computeTotalArea(circles);
    std::cout << "Area : " << total_area << std::endl;


    //3) Computes the perimeter of these circles
    //
    // We leave this for students to complete.


    return 0;
}
