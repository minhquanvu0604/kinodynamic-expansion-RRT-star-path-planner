#include "rectangle.h"
#include <cmath>

Rectangle::Rectangle(double width, double height):
width_(width), height_(height)
{
    double diff = std::fabs(height_- width_);
    if (diff<1e-6) {
        description_ = "square";
    } else {
        description_ = "rectangle";
    }
}

Rectangle::Rectangle():
width_(0.0), height_(0.0)
{
    description_="point";
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
    double diff = std::fabs(height_- width_);
    if (diff<1e-6) {
        description_ = "square";
    } else {
        description_ = "rectangle";
    }
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
