// We need to include the declaration of our new circle class in order to use it.
#include "circle.h"

#include <iostream>
#include <vector>
#include <random>
#include <chrono>

// Defining functions like this above main is not best practice
// We will talk later about where this function belongs
double computeTotalArea(std::vector<Circle> circles) {
    double total_area = 0;
    for (auto circle : circles) {
        total_area += circle.getArea();
    }


    return total_area;
}

int main () {

    // Create an empty vector of circles
    std::vector<Circle> circles;

    // Setup random number generation using seed from system clock
    // We set up the seed and the distribution once, and thereafter simply draw from this distribution.
    long seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine gen(seed);
    //https://en.cppreference.com/w/cpp/numeric/random/uniform_real_distribution
    std::uniform_real_distribution<double> dist(1.0, 10.0); //Produces random floating-point values i, uniformly distributed on the interval [a, b]  which for us is 1.0 to 10.0

    //How many circles should we create? Let's ask user
    int num_circles=0;
    std::cout << "How many circles do you wish to generate : ";
    // Ensure value is between 0 and ARRAY_MAX_SIZE
    bool inRange = false;
    while (!inRange) {
        while(!(std::cin >> num_circles)){
            std::cin.clear();
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            std::cout << "Invalid input, Try again: ";
        }
        if (num_circles <= 0) {
            std::cout << "Value must be over" << 0 << ", Try again: ";
        } else {
            inRange = true;
        }
    }
    std::cout << std::endl;


    // Loop through and add a bunch of rectangles with random (1.0-10.0) sides
    for (int i = 0; i < num_circles; i++) {
        circles.push_back(Circle(dist(gen))); // Here we draw from the distribution, Each call to dis(gen) generates a new random double
    }

    // Compute the total area and print it
    double total_area = computeTotalArea(circles);
    std::cout << "Area after: " << total_area << std::endl;

    return 0;
}
