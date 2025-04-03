#include "processing.h"

void removeLargerThanArea(std::vector<Shape*> &shapes, double limit){
    for (int i = 0; i < shapes.size();)
        if (((shapes.at(i))->getArea()) > limit)
            shapes.erase(shapes.begin() + i);
        else i++;
}
