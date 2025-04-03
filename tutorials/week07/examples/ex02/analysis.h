#ifndef ANALYSIS_H
#define ANALYSIS_H

#include <vector>
#include <thread>
#include <mutex>
#include <atomic>
#include "data.h"

class Analysis
{
public:
    Analysis(pfms::Data* data);
    ~Analysis();

    bool isRunning();

private:

    void processSamples();

    int fibonacci(int n);

    pfms::Data* data_; //!< The number to be incremented / printed
    std::atomic<bool> running_;//!< Indicates if the thread is running
    std::thread* thread_; //!< Thread for increment
};

#endif // ANALYSIS_H
