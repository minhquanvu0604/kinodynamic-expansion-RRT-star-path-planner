#include <iostream>
#include <vector>

#include "rectangle.h"
#include "circle.h"
#include "triangle.h"


using std::cout;
using std::endl;
using std::vector;

int main () {

    Rectangle rect(4.0,5.0);

    cout << rect.getDescription() << " area:" <<  rect.getArea() << endl;

    vector<Shape* > shapes;

    //! We are using new to directly get the pointer to the object
    //! Alternative is to create object first and then obtain the
    //! memory address via "&" address of operator
    shapes.push_back(new Rectangle(3.0,3.0));
    shapes.push_back(new Circle(3.0));
    shapes.push_back(new Triangle(3.0,3.0));

    //! Questions
    //!
    //! Is s->getDescription() and (*s).getDescription() the same

    for (auto s : shapes) {
        cout << s->getDescription() << " has area " <<  (*s).getArea() << endl;
    }


    //! An alternative is using shared pointers and emplace_back
    //  vector<std::shared_ptr<Shape> > shapes;
    //  shapes.emplace_back(new Circle(distribution(generator)));

}



