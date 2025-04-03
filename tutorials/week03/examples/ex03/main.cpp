#include <iostream>
#include <vector>
#include <sstream> // std::stringstream

#include "rectangle.h"
#include "circle.h"

using std::cout;
using std::endl;
using std::vector;

int main () {

    Rectangle rectangle;
    rectangle.setHeightWidth(5.0, 3.5);
    std::cout << "The area of rectangle is " << rectangle.getArea() << std::endl;
    std::cout << "It is a " << rectangle.getDescription() << std::endl;

    Circle circle(5.0);
    std::cout << "The area of our circle is " << circle.getArea() << std::endl;
    std::cout << "It is a " << circle.getDescription() << std::endl;
    circle.offset(9.0,9.0);

    double x=0.0,y=0.0;
    std::stringstream ss;
    ss << "Point [" << x << "," << y <<"] ";
    if (circle.checkIntercept(x,y)){
      ss << " is inside ";
    }
    else{
       ss << " is outside ";
    }
    ss  << "the " << circle.getDescription() << endl;

    //We have used the stringstream to concatante a number of outputs into one stream of data which we can then print to console
    cout << ss.str();


    //NOTES:
    //We can not solve question 4 at this stage without looking at virtual keyword and methods .... enter concept of polymorphism
    //and a pointer to a base class http://www.cplusplus.com/doc/tutorial/polymorphism/

    std::vector<Shape *> shapes;
    // new is a keyword that creates a pointer
    // for example a Rectangle* is created as per below
    //Rectangle* rect = new Rectangle(5.0,4.0);
    shapes.push_back(new Rectangle(5.0,4.0));
    shapes.push_back(new Circle(5.0));

    std::cout << "Going through collection" << std::endl;
    for (auto shape : shapes){
        std::cout << shape->getDescription() << std::endl;
        std::cout << shape->getArea() << std::endl;
    }



}
