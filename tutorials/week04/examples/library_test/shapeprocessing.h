#ifndef SHAPEPROCESSING_H
#define SHAPEPROCESSING_H

#include "shape.h"
#include "circle.h"
#include "rectangle.h"

#include <vector>

using std::vector;

namespace shapeprocessing {

  static const double max_size=500;
  static const double max_length=100;
}

class ShapeProcessing
{
public:
  ShapeProcessing(vector<Shape*> shape);

  /**
  This function indicates if the point intersects all of the shapes, removing
  the shapes that have been intercected from the internal shape container
  and keeping the queries points in an internal vector

  @param[in]    x position
  @param[in]    y position
  @param[out]   point intesects a shape
  */
  bool checkIntersections(double x, double y);

private:
  vector<Shape*> shape_; //!< internal container of vector of Shape*
  std::vector<double> x_;//!< internal container of x points queried
  std::vector<double> y_;//!< internal container of y points queried

};



#endif // SHAPEPROCESSING_H
