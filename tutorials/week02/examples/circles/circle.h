#ifndef CIRCLE_H
#define CIRCLE_H

// Declaration of the Circle class
class Circle {

// Public members are accessible from outside the class (ie. in main)
public:
    // Declare the constructor
    Circle();
    Circle(double radius);
    // Declare the set_values method
    void setRadius (double radius);
    // Declare the area method
    double getArea();
    //Declare perimeter method
    double getPerimeter();
    // Declare the set area method
    void setArea(double area);
    //Declare set perimeter method
    void setPerimeter(double perimeter);

// Private members are only accessible from within methods of the same class
private:
    void recalculateArea(); //! Recalculates area again from the stored radius
    void recalculatePerimeter(); //! Recalculates perimeter again from the stored radius


    // This class has a double to represent the radius of the Circle
    // The trailing underscore is used to differentiate the member varibles
    // ..from local varibles in our code, this is not compulsary but HIGHLY recommended
    double radius_;
    double area_; // The area
    double perimeter_; // The perimeter

};

#endif // CIRCLE_H
