#include "triangle.h"

Triangle::Triangle(double width, double height):
    width_(width), height_(height)
{
    description_ = "isoc triangle";
}

void Triangle::setHeightWidth(double width, double height)
{
    width_ = width;
    height_ = height;
}

double Triangle::getArea()
{
    return width_ * height_ * 0.5;
}

//! Shoudl this work for triangle?
bool Triangle::checkIntercept(double x, double y){
 return ((x>=centreX_-(width_/2)) &&
      (x<=centreX_+(width_/2)) &&
      (y<=centreY_+(height_/2)) &&
      (y>=centreY_-(height_/2)) &&
      (pow(x-width_/2, 2) + pow(y-height_, 2) <= pow(height_, 2) + pow(width_/2, 2) ));
}
