// Includes std::cout and friends so we can output to console
#include <iostream>

// Create a macro (#define) to represent the array size
#define ARRAY_SIZE 10

// Every executable needs a main function which returns an int
int main () {
    // Create an array x of doubles with 10 elements
    // Populate the elements of array on creating of array, each element [i] has value i (INITIALISER LIST)
    double x[ARRAY_SIZE] = {0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0};

    // Can you create a loop to populate elements of x (each x[i] =i), how to code end of loop?) – (USE MACRO)
    for (int i = 0; i<ARRAY_SIZE; i++) {
        x[i] = i;
        std::cout << "x[" << i << "] = " << x[i] << std::endl;
    }

    // Can you use a pointer and loop to initialise elements of array (*ip++)
    //
    // Two concepts here
    // 1) loop termination
    // ip is a double pointer, so the loop termination is when
    // ip points to a memory address which is x+ARRAY_SIZE, ip can't be comapred to ARRAY_SIZE
    // 2) the increment step
    // As we have to do assignment as well as incrementing pointer we use a trick
    // we assign to *ip ip-x which increases as we go through the array (x is always the zeroth element)
    // And we also incraese the ip pointer, the location it points to by one
    for (double *ip = x; ip<(x+ARRAY_SIZE); *ip++ = ip-x) {
        std::cout << "*ip = " << *ip << std::endl;
    }

    // Main function should return an integer
    return 0;
}
