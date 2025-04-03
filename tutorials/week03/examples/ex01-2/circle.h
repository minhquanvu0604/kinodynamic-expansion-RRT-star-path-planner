#ifndef CIRCLE_H
#define CIRCLE_H

#include "shape.h"

class Circle : public Shape
{
public:
    Circle(double radius);

    void setRadius(double radius);
    double getArea ();
private:
    double radius_;//radius

};

#endif // CIRCLE_H
