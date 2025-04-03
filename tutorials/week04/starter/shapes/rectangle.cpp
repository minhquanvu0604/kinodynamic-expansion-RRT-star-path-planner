#include "rectangle.h"
#include <limits>

Rectangle::Rectangle(double width, double height):
width_(width), height_(height)
{
  //! What does std::numeric_limits<double>::epsilon() do?
  //! https://en.cppreference.com/w/cpp/types/numeric_limits/epsilon
  if ((width_-height_)< std::numeric_limits<double>::epsilon()) {
      description_ = "square";
  } else {
      description_ = "rectangle";
  }
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
    if ((width_-height_)< std::numeric_limits<double>::epsilon()) {
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
