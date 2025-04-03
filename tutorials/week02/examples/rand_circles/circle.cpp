#include "circle.h"
#include <cmath>

Circle::Circle(): // This constructor can call a constructor that requires a parameter and supply a default parameter
    Circle(0.0)
{
}

Circle::Circle(double radius):
    radius_(radius)/// This is the best form of initialisation for a constructor, via an initialiser list
{
    recalculateArea();
    recalculatePerimeter();
}

void Circle::setRadius(double radius)
{
    radius_ = radius;
}

double Circle::getArea()
{
    return radius_ * radius_ * M_PI;
}

double Circle::getPerimeter()
{
    return 2 * M_PI *radius_;
}

void Circle::setArea(double area){
    area_=area;
    radius_= std::pow(area_/M_PI,0.5);
    recalculatePerimeter();
}
void Circle::setPerimeter(double perimeter){
    perimeter_ = perimeter;
    radius_= perimeter_/(2*M_PI);
    recalculateArea();
}

void Circle::recalculatePerimeter(){
    perimeter_ = M_PI*2*radius_;
}


void Circle::recalculateArea(){
    area_ = radius_*radius_*M_PI;
}

