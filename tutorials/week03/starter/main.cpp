#include <iostream>
#include "circle.h"

using std::cout;
using std::endl;

int main(){

    Circle circ;

    cout << circ.getDescription() << " has area:" << circ.getArea() << std::endl;

}
