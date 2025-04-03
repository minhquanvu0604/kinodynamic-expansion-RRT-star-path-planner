#include "analysis.h"
#include <iostream>

using std::cout;
using std::endl;

Analysis::Analysis() : num_(1)
{
    // Create the threads
    running_=true;
    incThread_ = new std::thread(&Analysis::incrementNum,this);
    printThread_ = new std::thread(&Analysis::printNum,this);
}

Analysis::~Analysis()
{
    running_=false;
    // Destruct the threads
    incThread_->join();
    printThread_->join();
    delete incThread_;
    delete printThread_;
}



void Analysis::incrementNum() {

    while (running_) {
        // We can only obtain a lock in this thread if the mutex
        // is not locked anywhere else
        {
            std::unique_lock<std::mutex> lck(numMutex_);
            // We only access num while the mutex is locked
            num_++;
        }
        std::this_thread::sleep_for (std::chrono::milliseconds(10));
    }
}

void Analysis::printNum() {
    while (running_) {
        {
            std::unique_lock<std::mutex> lck(numMutex_);
            // We only access num while the mutex is locked
            cout <<  "num is:" << num_ << endl;
        }
        // This delay is included to improve the readability of the program output
        // and avoid hard-looping and consuming too much cpu
        // A better solution would be to use a condition_variable to make
        // this thread wait for a notification from the incrementing thread
        std::this_thread::sleep_for (std::chrono::milliseconds(50));
    }
}