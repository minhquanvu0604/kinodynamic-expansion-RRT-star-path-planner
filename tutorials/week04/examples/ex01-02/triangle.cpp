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

void Triangle::setCentre(double x, double y)
{
    Shape::setCentre(x, y);
    updateEdges();
}

bool Triangle::checkIntercept(double x, double y){

//https://math.stackexchange.com/questions/324589/detecting-whether-a-point-is-above-or-below-a-slope

//https://www.quora.com/Where-is-the-centre-of-mass-of-an-isosceles-triangle
    return base_edge_.pointAboveLine(x, y)
           && !left_edge_.pointAboveLine(x, y)
           && !right_edge_.pointAboveLine(x, y);
}

void Triangle::updateEdges()
{
    base_edge_.setGradient(0.0);
    base_edge_.setYIntercept(centreY_ - 0.5 * height_);
    left_edge_.fromPoints(centreX_ - 0.5 * width_, centreY_ - 0.5 * height_,
                          centreX_,                centreY_ + 0.5 * height_);
    right_edge_.fromPoints(centreX_ + 0.5 * width_, centreY_ - 0.5 * height_,
                          centreX_,                centreY_ + 0.5 * height_);
}
