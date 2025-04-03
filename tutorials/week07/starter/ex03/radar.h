#ifndef RADAR_H      // An 'include guard' to prevent double declaration of any identifiers in this library
#define RADAR_H

#include <string>
#include <vector>
#include <random>
#include <atomic>
#include <mutex>
#include <condition_variable>
#include <thread>

class Radar{
    public:

    Radar();
    ~Radar();
    std::vector<double> getData(void);	            // Return a vector of targets, blocking call function
    double getScanningTime(void);                   // Get Scanning Time in ms
    void setScanningTime(double scanningTime);      // Set Scanning Time in ms
    void start();                                   // Start the thread that generates data
    double getMaxDistance(void);                    // Retruns Max Distance;
private:
    void generateData(void);
    std::vector<double> data_;                 //targets as a vector
    double scanningTime_;
    const double maxDistance_ = 80.0;
    const unsigned int numTarget_ = 20;
    std::mt19937 *generator_;
    std::uniform_real_distribution<double> *value_;
    std::atomic<bool> ready_;                     // Indicates if new data has been recieved
    std::mutex mtx_;
    std::condition_variable cv_;

    std::mutex mtxStart_;
    std::condition_variable cvStart_;

    std::vector<std::thread> threads_; // We add threads onto a vector here to be able to terminate then in estructor
    std::atomic<bool> running_;       // We use this to indicate the loop in data generation shoudl still be running

};

#endif // SENSOR_H
