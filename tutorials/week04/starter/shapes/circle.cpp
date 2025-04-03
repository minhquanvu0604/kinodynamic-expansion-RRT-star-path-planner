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



bool Circle::checkIntercept(double x, double y){

  return std::pow(x-centreX_,2) + std::pow(y-centreY_,2)<std::pow(radius_,2) ;

}
