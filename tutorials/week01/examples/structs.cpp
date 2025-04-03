// Includes std::cout and friends so we can output to console
#include <iostream>

//Why not #define ?
static const int max_size =100;

//http://www.cplusplus.com/doc/tutorial/structures/
struct Sensor{
    int num_samples;
    double data[max_size]; //Why have we picked this size here?
};
//Why do we move away from struct to classes
// - To protect data and ensure integirty (access specifiers)
// - To embed data and functionality together
// - To ensure the container is safe to use on ititialisation (constructor)

void print_struct(Sensor sensor){
  //! Loop to print elements of data in struct (let's acess each element location
    for (unsigned int i=0;i<sensor.num_samples;i++){
        std::cout << sensor.data[i] << " ";
    }
    std::cout << std::endl;
}


// Every executable needs a main function which returns an int
int main (int argc,char** argv) {

    //Ex03

    //Create a sensor with Sensor type structure
    Sensor sensor;

    //Let's say we have 5 samples ...
    sensor.num_samples=5;

    //Initialise the data
    for (unsigned int i = 0; i < sensor.num_samples; i++) {
      sensor.data[i] = i;
    }

    //Function called to print struct
    print_struct(sensor);

    //This is known as a list initalisation (aggregate initialisation)
    //https://en.cppreference.com/w/cpp/language/list_initialization
    //https://en.cppreference.com/w/cpp/language/aggregate_initialization
    Sensor sensor2 { 2, { 7, -12}}; //Create data of size two with elements noted

    //Function called to print struct
    print_struct(sensor2);


    return 0;
}




