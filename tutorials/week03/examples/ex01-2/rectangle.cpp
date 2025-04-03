#include "rectangle.h"
#include <limits>


Rectangle::Rectangle(double height, double width):
width_(width), height_(height)
{
    updateDescription();//we call a function to update the description
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
    updateDescription();//we call a function to update the description
}

double Rectangle::getArea(void)
{
    return width_ * height_;
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