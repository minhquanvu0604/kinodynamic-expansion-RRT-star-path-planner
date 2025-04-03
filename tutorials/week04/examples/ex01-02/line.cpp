#include "line.h"

Line::Line():
    gradient_(0.0),
    y_intercept_(0.0)
{
}

Line::Line(double gradient, double y_intercept):
    gradient_(gradient),
    y_intercept_(y_intercept)
{
}

Line::Line(double ax, double ay, double bx, double by)
{
    fromPoints(ax, ay, bx, by);
}

void Line::fromPoints(double ax, double ay, double bx, double by)
{
    gradient_ = (by - ay) / (bx - ax);
    y_intercept_ = ay - gradient_ * ax;
}

void Line::setGradient(double gradient)
{
    gradient_ = gradient;
}

void Line::setYIntercept(double y_intercept)
{
    y_intercept_ = y_intercept;
}

bool Line::pointAboveLine(double x, double y)
{
    double line_y = gradient_ * x + y_intercept_;
    return y > line_y;
}
