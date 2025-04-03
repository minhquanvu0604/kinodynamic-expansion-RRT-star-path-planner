#ifndef RECTANGLE_H
#define RECTANGLE_H

#include "shape.h"
/*!
 *  \brief     Rectangle class
 *  \details
 *  Rectangle class.\n
 *  \author    Alen Alempijevic
 *  \author    Alex Virgona
 *  \version   1.02-2
 *  \date      2016
 *  \pre       none
 *  \bug       none reported as of 2016-04-11
 *  \warning
 */
class Rectangle : public Shape
{
public:
    Rectangle(double width, double height);

    /**
    This function sets the width and height of a Rectangle

    @param[in]    width dimension of rectangle
    @param[in]    height dimension of rectangke
    */
    void setHeightWidth(double width, double height);
    
    //!ADDITIONAL QUESTIONS TO CONSIDER
    // Consider if getArea() is a method that should exist in Rectangle?
    // Should all shapes be able to computer Area? Do all shapes have this attribute?
    // A design to enable this is covered in when we introduce polymorphism
    double getArea (void) override;

    bool checkIntercept(double x, double y) override;

private:

    //!ADDITIONAL QUESTIONS TO CONSIDER
    // Why are these member varaibles in Rectangle, and not in shape?

    double width_; //!< width of rectangle
    double height_;//!< height of rectangle
};

#endif // RECTANGLE_H
