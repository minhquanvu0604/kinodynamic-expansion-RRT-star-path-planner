#ifndef SHAPE_H
#define SHAPE_H

#include <string>
#include <cmath>

/*!
 *  \ingroup   ac_shapre Shape
 *  \brief     Shape base class
 *  \details
 *  This class is the base class for all shapes.\n
 *  \author    Alen Alempijevic
 *  \author    Alex Virgona
 *  \version   1.02-1
 *  \date      2016
 *  \pre       none
 *  \bug       none reported as of 2016-04-11
 *  \warning
 */
class Shape
{
public:
    Shape();

    /**
    This function sets the centre position (x,y)

    @param[in]    x centre position
    @param[in]    y centre position
    \sa Shape() and offsetCentre()
    */
    void setCentre(double x, double y);

    /**
    This function offsets the centre position by (x,y)

    @param[in]    x centre position offset
    @param[in]    y centre position offset
    \sa Shape() and setCentre()
    */
    void offsetCentre(double x, double y);

    //! Returns the description of the shape
    /*!
      \return The description
    */
    std::string getDescription();

    //! Computes the area and returns value [m2].
    /*!
      \return The area
    */
    virtual double getArea()=0;

    //! Computes the perimeter and returns value [m2].
    /*!
      @param[out]    perimeter of rectangle
    */
    //double getPerimeter();

    /**
    This function indicates if the point intersects (is within) the shape

    @param[in]    x position
    @param[in]    y position
    @param[out]   point intesects shape
    */
    virtual bool checkIntercept(double x, double y)=0;

protected:
    std::string description_;//!< description of shape
    double centreX_;//!< X coordinate of centre of shape
    double centreY_;//!< Y coordinate of centre of shape
};

#endif // SHAPE_H
