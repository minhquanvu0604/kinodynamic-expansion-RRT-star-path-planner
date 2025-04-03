#include <iostream>
#include <limits>
#include <random>
#include <chrono>

#include "shapeprocessing.h" // Q) Why do we only need to include this header?

int main () {

  long seed = std::chrono::system_clock::now().time_since_epoch().count();
  std::default_random_engine gen(seed);

  vector<Shape*> shape;

  //    * Allow the user to specify number of circles and rectangles
  std::cout << "Specify the number of circles: ";
  std::cin >> num_cir;
  std::cout << std::endl;

  std::cout << "Specify the number of triangles: ";
  std::cin >> num_tri;
  std::cout << std::endl;

  std::cout << "Specify the number of rectangles: ";
  std::cin >> num_rec;
  std::cout << std::endl;

  std::cout << "Specify the max length: ";
  std::cin >> max_len;
  std::cout << std::endl;

  //    * Create the shapes with random lengths to be capped to `MAX_LENGTH` - which is a const in    [shape_processing.h]./starter/library_test/shape_processing.h).
  std::uniform_real_distribution<double> dist(1.0, max_len);

  
  for (int i = 0; i < num_cir; i++){
    radius = dist(gen);
    shape.push_back(new Circle(radius));
  }

  for (int i = 0; i < num_tri; i++){
    hei_tri = dist(gen);
    base_tri = dist(gen);
    shape.push_back(new Triangle(hei_tri, base_tri));
  }

  for (int i = 0; i < num_rec; i++){
    hei_rec = dist(gen);
    wid_rec = dist(gen);
    shape.push_back(new Rectangle(hei_rec, wid_rec));
  }



  //    * Allows user to enter a location x,y within (-MAX_SIZE, MAX_SIZE). `MAX_SIZE` is a const in [shape_processing.h]./starter/library_test/shape_processing.h) until all the shapes have been intersected


    //! What is this? Why is this a good practice?
    //! How has this been wrapped, look up namespace.
    std::cout << shapeprocessing::MAX_SIZE << std::endl;


    //! Additional questions
    //! Is shape and the shape_ in ShapeProcessing the same?
    //! If not, how do we change the code for this to occur?

    return 0;
}
