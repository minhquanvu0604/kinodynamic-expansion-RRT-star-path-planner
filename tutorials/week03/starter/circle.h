#ifndef CIRCLE_H
#define CIRCLE_H
#include "shape.h"

class Circle
{
public:
  Circle();
  Circle(double radius);
  // Declare the set_values method
  void setRadius (double radius);
  // Declare the area method


  //Declare perimeter method
  double getPerimeter();
  // Declare the set area method
  void setArea(double area);
  //Declare set perimeter method
  void setPerimeter(double perimeter);
private:
  double radius_;
  double area_; // The area
  double perimeter_; // The perimeter


};

#endif // CIRCLE_H
