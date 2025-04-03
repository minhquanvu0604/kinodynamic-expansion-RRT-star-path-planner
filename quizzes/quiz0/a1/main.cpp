#include <limits> //Needed to find max number
#include <iostream> // Includes std::cout and friends so we can output to console
#include "arrays.h"


const int array_max_size = 100;


// HELPER function to print elements of the array
void printArray(double *x, unsigned int xSize){
  for (unsigned int i = 0; i<xSize; i++) {
      std::cout << "x[" << i << "] = " << x[i] << std::endl;
  }
}


// HELPER function to print elements of the vector
void printVec(std::vector<double> myVec){

  for(unsigned int i=0;i<myVec.size();i++){
    std::cout << "[" << i << "] " << myVec.at(i) << std::endl ;
  }

}


int main () {

    unsigned int arraySize=0;
    // We create an array with max size
    double x[array_max_size];

    // Ask user to specify how many additional numbers are to be generated
    unsigned int num;
    do{
        std::cout << "How many random elements do you wish to generate (0 - "<< array_max_size << "): ";
         while(!(std::cin >> num)){
             std::cin.clear();
             std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
             std::cout << "Invalid input, Try again: ";
         }
    }
    while(num>array_max_size);

    // Populate array with random numbers
    populateWithRandomNumbers(x,arraySize,num);

    // Print out contents of array
    std::cout << "Initial array" << std::endl;
    printArray(x,arraySize);

    //Let's create a vector of doubles
    std::vector<double> myVec;

    //Uncomment below to assign the array to a vector ( TASK 1 )
   assignArrayToVector(x,arraySize,myVec);

    //Uncomment below to compute mean and Standard Deviation ( TASK 2 )
   Stats stats = computeMeanAndStdDev(myVec);

    //Uncomment below section to remove numbers larger than ( TASK 3 )
   removeNumbersLargerThan(myVec,stats.mean+(2*stats.std_dev));
   std::cout << "vector with elements over (" << stats.mean  + 2 * stats.std_dev << ") removed" << std::endl;
   printVec(myVec);

    // Uncomment below section to obtain vector of elements less than ( TASK 4)
   std::vector<double> myVecSmall = returnVecWithNumbersSmallerThan(myVec,stats.mean-(2*stats.std_dev) );
   std::cout << "vector with elements under (" << stats.mean  - 2*  stats.std_dev << ") " << std::endl;
   printVec(myVecSmall);


    // Main function should return an integer
    return 0;
}
