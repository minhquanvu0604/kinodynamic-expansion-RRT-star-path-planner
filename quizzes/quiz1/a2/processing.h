#ifndef PROCESSING_H
#define PROCESSING_H
#include <vector>
#include "shape.h"

/**
 * @brief Remove shapes from the vector that have an area larger than specific limit
 * @param shapes in|out vector os Shape* to be processed
 * @param limit in shapes over this limit in terms of area need to be removed
 */
void removeLargerThanArea(std::vector<Shape*> &shapes, double limit);


#endif // PROCESSING_H
