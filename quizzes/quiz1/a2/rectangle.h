#ifndef RECTANGLE_H
#define RECTANGLE_H

#include "shape.h"

//1) TASK
//Modify the file rectangle [rectangle](./a2/rectangle.h) so it inherits from the base class of shape [shape](./a2/shape.h)

class Rectangle : public Shape
{


public:
//    TASK 1
//    The `Rectangle` class already has a special member function.
//    Thanks to *Polymorphism* we can have functions with same name but different number of parameters.
//    Add to the `Rectangle` class another function that enables the `Rectangle` to on creation have `width` and `height` initialised with values supplied by user of the class


    Rectangle();
    Rectangle(double width, double height);
    /**
     * @brief Function that sets width and height
     * @param width in [m]
     * @param height in [m]
     */
    void setHeightWidth(double width, double height);

    // make getArea() a method that
    // 1) Base class Shape defines
    // 2) Needs to be in Rectangle
    // 3) makes Shape a pure virtual class
    /**
     * @brief Function that returns area
     * @return area in [m2]
     */
    double getArea (void) override;
private:

    double width_; //!< width of rectangle
    double height_;//!< height of rectangle
};

#endif // RECTANGLE_H
