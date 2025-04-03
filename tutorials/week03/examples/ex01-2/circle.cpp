#include "circle.h"

Circle::Circle(double radius) :
radius_(radius)
{
  description_="circle";
}

void Circle::setRadius(double radius){
    radius_=radius;

}

double Circle::getArea (){
    return std::pow(radius_,2)*M_PI;

}

