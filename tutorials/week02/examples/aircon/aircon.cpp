#include "aircon.h"
#include <thread>
#include <cmath>
#include <random>
#include <sstream>

double x=0;

Aircon::Aircon() :
    state_(aircon::State::IDLE), heating_active_(false), run_time_(3*60)
{
    long seed = std::chrono::system_clock::now().time_since_epoch().count(); //This is a seed for our random number generator, otherwise would always have the same "random" number when it starts
    std::default_random_engine generator (seed);
    std::uniform_real_distribution<double> distribution(1.0,50.0);// unfirom distrbution betwen 1 to 50
    current_temp_ = distribution(generator);

    //Let's get current time (needed to sample the temperature as well as terminate
    //After the duration it will run
    start_time_ = system_clock::now();
}



// Determine the elapsed time in s
double Aircon::elapsedTime(){
    return (duration_cast<milliseconds>(system_clock::now() - start_time_).count())/1000;
}

// Sleep for specified duration in ms
void Aircon::   sleepMs(int ms){
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

// Heating (would result in increase of room temperature)
void Aircon::heating(){
    current_temp_+=0.5;
}

// Cooling (would result in decrease of room temperature)
void Aircon::cooling(){
    current_temp_-=0.5;
}

// Check if temperature reached
bool Aircon::tempReached (){
    //We check wether the temperature is within tolerance
    //You CAN NOT compare floats or doubles
    //it ALWAYS needs to be done to a tolerance
    return std::fabs(current_temp_-desired_temp_) < 0.5;
}


bool Aircon::setDesiredTemp(double temp){
    bool OK = false;
    if(temp>10 && temp<50){
        desired_temp_=temp;
        OK = true;
    }
    return OK;
}

double Aircon::getCurrentTemp(){
    return current_temp_;
}

void Aircon::setRunTime(int timeS){
    run_time_ = timeS;
}


bool Aircon::run(){

    if((elapsedTime()> run_time_) && (state_ == aircon::State::IDLE)){
        return true;
    }

    switch(state_){
        case aircon::State::IDLE : {
            if((current_temp_-desired_temp_)>0.5){
                state_=aircon::State::COOLING;
            }
            else if((desired_temp_-current_temp_)>0.5){
               state_=aircon::State::HEATING;
            }
            break;
        }
        case aircon::State::HEATING : {
            if (tempReached()){
                state_=aircon::State::IDLE;
                break;
            }
            if(heating_active_){
                heating();
            }
            else{
                heating_active_=true;
            }
            break;
        }
        case aircon::State::COOLING : {
            if (tempReached()){
                state_=aircon::State::IDLE;
                break;
            }
            cooling();
            break;
        }
    }

    sleepMs(500);//Sleep for 0.5s (in ms)

    if((elapsedTime()> run_time_) && (state_ == aircon::State::IDLE)){
        return true;
    }

    return false;

}



std::string Aircon::getInfoString()
{
    std::stringstream ss;
    ss << elapsedTime() << " ";

    switch(state_)
    {
        case aircon::State::IDLE   : ss << "IDLE ";    break;
        case aircon::State::HEATING : ss << "HEATING ";  break;
        case aircon::State::COOLING : ss << "COOLING ";  break;
    }
    ss << "current t=" << current_temp_ ;
    return ss.str(); // This command convertes trsingstream to string
}
