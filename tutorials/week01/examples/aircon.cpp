// Includes std::cout and friends so we can output to console
#include <iostream>
// Random number
#include <random>
// thread and chrono are for time and sleeping respectively
#include <chrono>
#include <thread>

using namespace std::chrono;//Let's us refer to system_clock without stating std::chrono

//Question to reflect on - Shoudl we define a tempertaure tolerance? (for our controller,
//and if so, what would this be related to)?

// Aircon unit has a few states
namespace aircon{
    enum State
    {
        IDLE,
        COOLING,
        HEATING
    };
}

//We create a structure that has all the variables belonging to the AirCon
struct AirCon{
    aircon::State state;
    bool heating_active;
    double desired_temp;
    double current_temp;
};

// Determine the elapsed time in ms
double elapsedMs(system_clock::time_point start_time){
    //We compute time between current time and supplied start time, and return elapsed Ms
    return duration_cast<milliseconds>(system_clock::now() - start_time).count();
}

// Sleep for specified duration in ms
void sleepMs(int ms){
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

// Heating (would result in increase of room temperature)
void heating(AirCon& aircon){
    aircon.current_temp+=0.5;
}

// Cooling (would result in decrease of room temperature)
void cooling(AirCon& aircon){
    aircon.current_temp-=0.5;
}

//We can print a summary of the aircon structure as we created a << operator for it
//For an introduction to thsi refer enum_example in Week 01 Pre-work
std::ostream& operator<<(std::ostream& os, AirCon aircon)
{

    switch(aircon.state)
    {
        case aircon::State::IDLE   : os << "IDLE ";    break;
        case aircon::State::HEATING : os << "HEATING ";  break;
        case aircon::State::COOLING : os << "COOLING ";  break;
    }
    os << "current t=" << aircon.current_temp << " desired t=" << aircon.desired_temp << " ";
    return os;
}

// Check if temperature reached
bool tempReached (AirCon aircon){
    //We check wether teh temperature is within 0.01C
    // you CAN NOT compare floats or doubles, it ALWAYS needs to be done to a tolerance
    return std::fabs(aircon.current_temp-aircon.desired_temp) < 0.5;
}


// Every executable needs a main function which returns an int
int main (int argc,char** argv) {

    AirCon aircon;
    aircon.state = aircon::State::IDLE; // We need to assign IDLE here on startup, the role for constructors will be discussed in Week 02
    aircon.heating_active=false;

    //! Obtain value of desired temperature
    std::cout << "What is the desired temperature? (1-50)" << std::endl;
    std::cin >>  aircon.desired_temp;
    std::cout << "Desired temperature entered:" << aircon.desired_temp << std::endl;

    long seed = std::chrono::system_clock::now().time_since_epoch().count(); //This is a seed for our random number generator, otherwise would always have the same "random" number when it starts
    std::default_random_engine generator (seed);
    std::uniform_real_distribution<double> distribution(1.0,50.0);// unfirom distrbution betwen 1 to 50

    aircon.current_temp = distribution(generator);

    std::cout << "Current temperature:" << aircon.current_temp << std::endl;

    //Let's get current time (needed to sample the temperature as well as terminate
    //After the duration it will run
    system_clock::time_point start_time = system_clock::now();
    double run_time = 3*60*1000; //3mins to miliseconds

    std::cout << "run time=" << run_time/1000 <<  " [s]" << std::endl;


    do{
        switch(aircon.state){
            case aircon::State::IDLE : {
                if((aircon.current_temp-aircon.desired_temp)>0.5){
                    aircon.state=aircon::State::COOLING;
                }
                else if((aircon.desired_temp-aircon.current_temp)>0.5){
                   aircon.state=aircon::State::HEATING;
                }
                break;
            }
            case aircon::State::HEATING : {
                if (tempReached(aircon)){
                    aircon.state=aircon::State::IDLE;
                    break;
                }
                if(aircon.heating_active){
                    heating(aircon);
                }
                else{
                    aircon.heating_active=true;
                }
                break;
            }
            case aircon::State::COOLING : {
                if (tempReached(aircon)){
                    aircon.state=aircon::State::IDLE;
                    break;
                }
                cooling(aircon);
                break;
            }
        }
        std::cout << aircon << std::endl;
        sleepMs(500);//(5*1000);//Sleep for 5s (in ms)
    }
    while ((elapsedMs(start_time)< run_time) || (aircon.state != aircon::State::IDLE));


    return 0;
}
