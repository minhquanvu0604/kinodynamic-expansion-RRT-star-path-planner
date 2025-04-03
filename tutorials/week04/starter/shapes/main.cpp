#include <iostream>
#include <vector>

#include "rectangle.h"
#include "triangle.h"
#include "circle.h"

using std::cout;
using std::endl;
using std::vector;

int main () {

    vector<Shape*> shapes;

    Rectangle r1 = Rectangle(2, 5);
    Triangle t1(4, 7);
    Circle c1(3);


    shapes.push_back(&r1);
    // WHY CAN'T I DO THIS ????    shapes.push_back(&(Rectangle r1 = Rectangle(2, 5)));
    
    shapes.push_back(&t1);
    shapes.push_back(&c1);    

    for(int i=0; i<shapes.size(); i++)
        std::cout << (shapes.at(i))->getArea() << std::endl;

    return 0;
}
