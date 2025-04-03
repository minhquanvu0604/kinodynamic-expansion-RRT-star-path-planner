#include "rectangle.h"

Rectangle::Rectangle(double width, double height):
width_(width), height_(height)
{
    updateDescription();
}

Rectangle::Rectangle():
  Rectangle(0.0,0.0)
{
}

void Rectangle::setHeightWidth(double width, double height)
{
    //!NOTES 
    // This is a example of why you should not allow direct access to member variables (why they are private)
    // Given we have a function to set the member varaibles, we also can leverage this function to set any
    // other member variables required, of perform any other operations that are needed to be executed
    // (such as invoking other methods)

    width_ = width;
    height_ = height;
    
    updateDescription();
}

double Rectangle::getArea(void)
{
    return width_ * height_;
}

bool Rectangle::checkIntercept(double x, double y){

 return ((x>=centreX_-(width_/2)) &&
      (x<=centreX_+(width_/2)) &&
      (y<=centreY_+(height_/2)) &&
      (y>=centreY_-(height_/2)) );
}

void Rectangle::updateDescription(void){
    //When doing a comparison of float/double numbers, we need a tolerance
    //Sometimes we have a specified tolerance, here we imply a very small difference
    if(std::fabs(height_-width_)<1e-6){
    description_="square";
    }
    else{
        description_="rectangle";
    }

}