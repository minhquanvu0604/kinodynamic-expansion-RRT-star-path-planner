#include <iostream>
#include <vector>
#include <sstream> // for stringstream

#include <random>   // Includes the random number generator
#include <ctime>    // Used to print timestamp along with data
#include <memory>

#include "rectangle.h"
#include "triangle.h"
#include "circle.h"

using std::cout;
using std::cin;
using std::endl;
using std::vector;

int main () {

    vector<Shape* > shapes;

    int numTriangles=0,numRectangles=0,numCircles=0;

    cout << "Enter number of circles:" << endl;
    cin >> numCircles;
    cout << "Enter number of triangles:" << endl;
    cin >> numTriangles;
    cout << "Enter number of rectangles:" << endl;
    cin >> numRectangles;

    double maxLength=0;
    cout << "Enter maximum length:" << endl;
    cin >> maxLength;

    //Let's use a random number generator for the legths
    std::random_device generator;
    std::uniform_real_distribution<double> distribution(0.0,maxLength);
    std::uniform_real_distribution<double> distributionPosition(0.0,maxLength/2.0);

    for(long int num=0;num<numTriangles;num++){
      shapes.push_back(new Triangle(distribution(generator), distribution(generator)));
      std::cout << shapes.at(shapes.size()-1)->getDescription() << " #" << num << std::endl;
    }

    for(long int num=0;num<numRectangles;num++){
      shapes.push_back(new Rectangle(distribution(generator), distribution(generator)));
      std::cout << shapes.at(shapes.size()-1)->getDescription() << " #" << num << std::endl;
    }

    for(long int num=0;num<numCircles;num++){
      shapes.push_back(new Circle(distribution(generator)));
      std::cout << shapes.at(shapes.size()-1)->getDescription() << " #" << num << std::endl;
    }

    double totalArea=0;
    for (auto s : shapes) {
        cout << s->getDescription() << " has area " <<  (*s).getArea() << endl;
        s->setCentre(distributionPosition(generator),distributionPosition(generator));
        totalArea+=s->getArea();
    }

    cout << "Total area is :" << totalArea << endl;

//    * Implement the checkIntercept funtion for [Isosceles_triangle](https://en.wikipedia.org/wiki/Isosceles_triangle) in `triangle.cpp`
//    * Allow the user to specify a point to be used for intercept checking `x` and `y`
    double x=0,y=0;

    cout << "Enter position x y, example 0.1 0.2" << endl;
    cin >> x >> y;
    for (auto s : shapes) {
        std::stringstream ss;
        bool intercept = s->checkIntercept(x,y);
        ss << s->getDescription();
        if(intercept){
            ss << " intercepts ";
        }
        else{
            ss << " does not intercept ";
        }
        ss << " point [" << x << " " << y << "]";
        cout << ss.str() << endl;
    }


//    * Write a function that check if  the shapes intersect the point

}

