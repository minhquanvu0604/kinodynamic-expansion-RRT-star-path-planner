
#include <iostream>
#include <limits>
#include <random>

#include "shapeprocessing.h" // Q) Why do we only need to include this header?

using std::cout;
using std::endl;

int main () {

  vector<Shape*> shapes;

  //    * Allow the user to specify number of circles and rectangles

  int numCircles,numRectangles;

  cout << "Enter number of circles" << endl;
  std::cin >>numCircles ;

  cout << "Enter number of rectangles" << endl;
  std::cin >>numRectangles;

  //    * Create the shapes with random lengths to be capped to `max_length` - which is a const in    [shape_processing.h]./starter/library_test/shape_processing.h).

  std::random_device generator;
  std::uniform_real_distribution<double> distribution(0.0001,shapeprocessing::max_length);

  for(int num=0;num<numRectangles;num++){
    shapes.push_back(new Rectangle(distribution(generator), distribution(generator)));
  }

  for( int num=0;num<numCircles;num++){
    shapes.push_back(new Circle(distribution(generator)));
  }

  //! What is this? Why is this a good practice?
  //! How has this been wrapped, look up namespace.
  std::cout << shapeprocessing::max_size << std::endl;

  ShapeProcessing shapeProcessing(shapes);

  //Allows user to enter a location x,y within (-max_size, max_size). `max_size` is a const in [shape_processing.h]./starter/library_test/shape_processing.h) until all the shapes have been intersected

  double x=shapeprocessing::max_size+1;
  double y=x;

  std::cout << "You will need to enter x,y of a point,(within:" << shapeprocessing::max_size <<
               ") until all " << shapes.size() <<
               " shapes are interected:" << std::endl;
  //bool sunkAll=false;
  while(1) {
    std::cout << "Please enter size of x:";
    std::cin >> x;
    std::cout << "Please enter size of y:";
    std::cin >> y;
    bool sunkAll = shapeProcessing.checkIntersections(x,y);
    if(sunkAll){
      std::cout << "Finished all shapes" << std::endl;
       break;
    }
  }



    //! Additional questions
    //! Is shape and the shape_ in ShapeProcessing the same?
    //! If not, how do we change the code for this to occur?

    return 0;
}
