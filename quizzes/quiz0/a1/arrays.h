#ifndef ARRAYS_H
#define ARRAYS_H

#include <vector>

//A structure for some variables, as they are tied to same entity
struct Stats{
    double mean;    //! mean
    double std_dev; //! standard deviation
};

// function to populate array with random numbers
void populateWithRandomNumbers(double x[], unsigned int& array_size, unsigned int num_elements);

/**
 * @brief Function that assigns all elements of array to vector
 * @param x input vector
 * @param arraySize size of array x
 * @param myVec vector to be returned
 */
void assignArrayToVector(double x[], unsigned int arraySize, std::vector<double> &myVec);

/**
 * @brief Function that removes elements of vector greater than the limit (value)
 * @param myVec vector to be returned
 * @param limit elements of vector greater than this value are removed
 */
void removeNumbersLargerThan(std::vector<double> &myVec, double limit);

/**
 * @brief Function that computes ands returns the means and standard deviation of a vector
 * @param myVec vector used
 * @return Stats structure containing the mean and stanrad deviation
 */
Stats computeMeanAndStdDev(std::vector<double> myVec);

/**
 * @brief Function that retruns a vector containing elements of an initial vector that are less than a value
 * @param myVec vector to be searched
 * @param elements of vector less than this value are removed
 * @return vector containing elements found
 */
std::vector<double> returnVecWithNumbersSmallerThan(std::vector<double> myVec, double limit);

#endif // ARRAYS_H
