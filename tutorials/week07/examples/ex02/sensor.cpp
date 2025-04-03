#include "sensor.h"
#include <iostream>

using std::cout;
using std::endl;

Sensor::Sensor(pfms::Data* data) :
    data_(data)
{
    // Create the thread
    running_=true;
    thread_ = new std::thread(&Sensor::generateSamples,this);
}

Sensor::~Sensor()
{
    running_=false;
    // Destruct the threads
    thread_->join();
    delete thread_;
}

// The function generates samples
void Sensor::generateSamples() {

  //Setup and seed our random normal distribution generator
  std::default_random_engine generator(std::chrono::duration_cast
                                       <std::chrono::nanoseconds>
                                       (std::chrono::system_clock::now().time_since_epoch()).count());
  std::normal_distribution<> distribution{20, 10}; //generates integeres, mean of 20 and a stdev of 10

  while (true) {

        // We can only obtain a lock in this thread if the mutex
        // is not locked anywhere else
        data_->mtx.lock();

        cout << "sample" << endl;
        // We only access num while the mutex is locked
        int sample = distribution(generator);
        data_->data.push_back(sample);

        //We can unlock here to free up access to mutex
        data_->mtx.unlock();
        data_->cv.notify_one();

        // This delay is included to improve the emulate some other process of generating the data
        // by the sensor which could be at a specific rate (maybe 10Hz - 100ms)
        std::this_thread::sleep_for (std::chrono::milliseconds(100));

    }
}