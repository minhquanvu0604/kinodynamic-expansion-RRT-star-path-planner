#include "circle.h"

#include "circle.h"
#include <cmath>

Circle::Circle(): // This constructor can call a constructor that requires a parameter and supply a default parameter
    Circle(0.0)
{
}

Circle::Circle(double radius):
    radius_(radius)
{
    description_="circle";
}

void Circle::setRadius(double radius)
{
    radius_ = radius;
}

double Circle::getPerimeter()
{
    return 2 * M_PI *radius_;
}

void Circle::setArea(double area){
    radius_= std::pow(area_/M_PI,0.5);
}
void Circle::setPerimeter(double perimeter){
    radius_= perimeter_/(2*M_PI);
}
