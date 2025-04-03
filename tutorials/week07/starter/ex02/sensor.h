#ifndef SENSOR_H
#define SENSOR_H

#include <thread>
#include <atomic>
#include <random>    // random number generation
#include "data.h"

class Sensor
{
public:
    Sensor(pfms::Data* data);
    ~Sensor();

private:
    void generateSamples();

private:    
    pfms::Data* data_; //!< The number to be incremented / printed
    std::atomic<bool> running_;//!< Indicates if the thread is running
    std::thread* thread_; //!< Thread for increment
};

#endif // Sensor_H
