#ifndef TRIANGLE_H
#define TRIANGLE_H

#include "shape.h"
#include "line.h"

class Triangle : public Shape
{
public:
    Triangle(double width, double height);
    void setHeightWidth(double width, double height);
    double getArea ();
    bool checkIntercept(double x, double y);
    void setCentre(double x, double y);

private:
    void updateEdges();

private:
    double width_; //!< width of triangle
    double height_;//!< height of triangle
    Line left_edge_;
    Line right_edge_;
    Line base_edge_;
};


#endif // TRIANGLE_H
