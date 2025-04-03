#include "shape.h"

Shape::Shape():
    description_("unknown shape"),centreX_(0.0),centreY_(0.0)
{
}

void Shape::setCentre(double x, double y)
{
    centreX_=x;
    centreY_=y;
}


std::string Shape::getDescription()
{
    return description_;
}

void Shape::offset(double x, double y)
{
    centreX_ += x;
    centreY_ += y;
}

