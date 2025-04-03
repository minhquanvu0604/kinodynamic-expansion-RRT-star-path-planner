// Includes std::cout and friends so we can output to console
#include <iostream>

// Create a macro (#define) to represent the array size
#define ARRAY_SIZE 10

// Every executable needs a main function which returns an int
int main()
{

    // Create an array x of doubles with 10 elements
    double a[ARRAY_SIZE];

    // Populate the elements of array on creating of array, each element [i] has value i (HINT: INITIALISER LIST)
    // double a[ARRAY_SIZE] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10};

    // Can you create a loop to populate elements of x (each x[i] =i), how to code end of loop?) – (HINT: USE MACRO)
    // for (int i = 0; i < ARRAY_SIZE; i++)
    // a[i] = i;

    // Can you use a pointer and loop to initialise elements of array
    double *b = a;

    for (double* i = a; i < a + ARRAY_SIZE; ++i)
    {
        *i = i - a;
    }

    for (int i = 0; i < ARRAY_SIZE; i++)
        std::cout << a[i] << std::endl;
        
    // Main function should return an integer
    return 0;
}
