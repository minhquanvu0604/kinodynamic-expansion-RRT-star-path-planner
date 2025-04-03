#ifndef CONTAINER_OPS_H
#define CONTAINER_OPS_H

#include <deque>


//!TODO TASK 1:
//! Create a function that accepts a container and modifies it by adding user specified numbers of elements
//! To front of container. The actual element is also supplied by user (for instance num_values =4 element =-1.5  means
//! four elements of -1.5 are added to begining of deque)
/**
 * @brief accepts a container and modifies it by adding user specified numbers of elements to the begining of the
 * container.
 * @param container to be modified
 * @param num_vales number of identical elements to be added
 * @param elelemt the value to be added
 */
void populateContainer(std::deque<double>& container, unsigned int num_values, double element);

//!TODO TASK 2: Create a function that accept the chosen container and rearranges elements by bubble sort operation
//! An example on arrays with C++ code is here https://www.programiz.com/dsa/bubble-sort
void bubbleSortContainer( std::deque<double>& container);

#endif // CONTAINER_OPS_H
