#ifndef RECTANGLE_H
#define RECTANGLE_H

#include "shape.h"

class Rectangle : public Shape
{
public:
    Rectangle();
    Rectangle(double width, double height);
    /**
     * @brief Function that sets width and height
     * @param width in [m]
     * @param height in [m]
     */
    void setHeightWidth(double width, double height);

    /**
     * @brief Function that returns area
     * @return area in [m2]
     */
    double getArea (void);

    /**
     * @brief Function that checks if shape intercepts a point
     * @param point location x in [m]
     * @param point location y in [m]
     */
    bool checkIntercept(double x, double y);//! return true if the point is within the rectangle


private:

    //!ADDITIONAL QUESTIONS TO CONSIDER
    // Why are these member varaibles in Rectangle, and not in shape?
    double width_; //!< width of rectangle
    double height_;//!< height of rectangle
};

#endif // RECTANGLE_H
