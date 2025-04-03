#include "shapeprocessing.h"
#include <iostream>
#include <algorithm>

ShapeProcessing::ShapeProcessing(vector<Shape*> shape):
  shape_(shape)
{
}

bool ShapeProcessing::checkIntersections(double x, double y){

  //! Example 1: Using an iterator
//  auto it = shape_.begin();
//  while (it != shape_.end()){
//    if( ((*it)->checkIntercept(x,y))){
//        std::cout << "Remove: " << (*(*it)).getDescription() << std::endl;
//         shape_.erase(it);
//    }
//    else{
//      ++it;
//    }
//  }


  //! Example 2: using .at() and while loop
  if(shape_.size()>0){
    unsigned int idx=0;
    while (idx<shape_.size()){
      if( (shape_.at(idx)->checkIntercept(x,y))){
          std::cout << "Remove: " << shape_.at(idx)->getDescription() << std::endl;
          shape_.erase(shape_.begin()+idx);
      }
      else {
        idx++;
      }
    }
  }

  //! Example 3: ONLY for the die hard c++ fans : using remove_if for STL (available in algorithm)
  //! and a lambda function
  //!
  //! remove_if
  //!
  //! template< class ForwardIt, class UnaryPredicate >
  //! ForwardIt remove_if( ForwardIt first, ForwardIt last, UnaryPredicate p );
  //! https://en.cppreference.com/w/cpp/algorithm/remove
  //!
  //! lambda function
  //!
  //! lambda function - which is a unnamed function
  //! [ captures ] ( params ) { body }
  //! https://en.cppreference.com/w/cpp/language/lambda
  //!
//  shape_.erase(std::remove_if(shape_.begin(),
//                            shape_.end(),
//                            [x,y](Shape* shape){return shape->checkIntercept(x,y);}),
//             shape_.end());

  return (shape_.size()==0);
}
