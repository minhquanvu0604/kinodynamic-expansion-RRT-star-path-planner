#ifndef ANALYSIS_H
#define ANALYSIS_H

#include <vector>
#include <thread>
#include <mutex>
#include <atomic>

class Analysis
{
public:
    Analysis();
    ~Analysis();

    void incrementNum();
    void printNum();

private:
    int num_; //!< The number to be incremented / printed
    std::atomic<bool> running_;//!< Indicates if the thread is running
    std::mutex numMutex_; //!< Mutex for num_
    std::thread* incThread_; //!< Thread for increment
    std::thread* printThread_; //!< Thread for decrement
};

#endif // ANALYSIS_H
