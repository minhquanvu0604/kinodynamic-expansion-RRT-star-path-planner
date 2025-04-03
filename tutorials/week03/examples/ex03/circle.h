#ifndef CIRCLE_H
#define CIRCLE_H

#include "shape.h"

class Circle : public Shape
{
public:
    Circle(double radius);

    void setRadius(double radius);
    double getArea ();

    bool checkIntercept(double x, double y);//! return true if the point is within the circle

private:
    double radius_;//radius

};

#endif // CIRCLE_H
