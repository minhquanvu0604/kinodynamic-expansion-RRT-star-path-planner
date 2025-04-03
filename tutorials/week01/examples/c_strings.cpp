// Includes std::cout and friends so we can output to console
#include <iostream>

// Every executable needs a main function which returns an int
int main () {

    // Create an string array (char[]) x to value “41012”
    // do we need to specify size on initialisation?
    // ...Nope, if initial values are given they are used to determine the size
    char x[] = "41012";

    // Initialise with (INITIALISER LIST)
//    char x[] = {'4', '1', '0', '1', '2', 0}; // Notice the zero

    // Can you create a loop to show elements x[i] (how to code end of loop, can we have a check for NULL, what is x[i]?)
    std::cout << "x = \"";
    for (int i = 0; x[i]!=0; i++) {
        std::cout << x[i];
    }
    std::cout << "\"" << std::endl;

    // Can you use a pointer and loop to print elements (*ip++)
    // Can you create a loop to show elements x[i] (how to code end of loop, can we have a check for NULL, what is x[i]?)
    std::cout << "x = \"";
    for (char *ip = x; *ip!=0; std::cout << *ip++) {}
    std::cout << "\"" << std::endl;

    // ADVANCED, Why does x[i]=NULL work for char[] for termination of loop, and will not work for double[]
    // C strings are NULL terminated, this works because 0 does not represent a valid character
    // double arrays are not NULL terminated, using 0 to mark the end would be silly because 0 could be a value of a number in the array

    // Main function should return an integer
    return 0;
}
