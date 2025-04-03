#ifndef TRIANGLE_H
#define TRIANGLE_H

#include "shape.h"

/*!
 *  \brief     Isosceles Triangle class
 *  \details
 *  Isosceles Triangle class.\n
 *  \author    Alen Alempijevic
 *  \author    Alex Virgona
 *  \version   1.02-2
 *  \date      2016
 *  \pre       none
 *  \bug       none reported as of 2016-04-11
 *  \warning
 */
class Triangle : public Shape
{
public:
    Triangle(double width, double height);

    /**
    This function sets the width and height of the Isosceles triangle

    @param[in]    width dimension of triangle
    @param[in]    height dimension of triangle
    */
    void setHeightWidth(double width, double height);

    bool checkIntercept(double x, double y) override;

    double getArea ();
private:
    double width_; //!< width of triangle
    double height_;//!< height of triangle
};


#endif // TRIANGLE_H
