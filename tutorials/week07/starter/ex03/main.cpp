#include <iostream>
#include "radar.h"

int main (void){

  // instantiate radar object
  Radar radar;

  while (true){
    std::vector <double> data = radar.getData();

    for(auto elem : data){
      std::cout << elem << " ";
    }
    std::cout << std::endl;
  }

  return 0;
}

