#ifndef DATABUFFER_H
#define DATABUFFER_H

#include <string>
#include <vector>
#include <mutex>
#include <atomic>
#include <thread>

using std::vector;

/*!
 * \brief     DataBuffer Class
 * \details
 * This class demonstrates how to create thread safe class, the enter user now does not have to deal
 * with mutexe's as the functions internally handle this.
 */
class DataBuffer
{
public:
    DataBuffer(double minVal, double maxVal,unsigned int trimSize);
    ~DataBuffer();

    /**
    Member function to remove values from internal container, uses minVal_ and maxVal_
    @multithreading This method is thread-safe because it internally handles a mutex
    */
    void removeValues();
    /**
    Member function to add values to internal container and sorts values
    @multithreading This method is thread-safe because it internally handles a mutex
    */
    void addValues(vector<double> values);
    /**
    Member function to trim internal container to specified size
    @multithreading This method is thread-safe because it internally handles a mutex
    */
    void trimLength();
    /**
    Member function to obtain values to internal container
    @multithreading This method is thread-safe because it internally handles a mutex
    */
    vector<double> getValues(void);

private:

    void bubbleSort(vector<double>& vec);

private:
    vector<double> val_;

    std::atomic<bool> running_;//!< Indicates if the thread is running
    std::mutex mtx_;
    double minVal_;
    double maxVal_;
    unsigned int trimSize_;

    std::thread* removeValuesThread_;
    std::thread* trimLengthThread_;

};

#endif // DATABUFFER_H
