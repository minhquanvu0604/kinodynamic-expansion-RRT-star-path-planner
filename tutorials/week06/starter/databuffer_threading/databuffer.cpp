#include "databuffer.h"
#include <random>
#include <chrono>
#include <thread>
#include <iostream>

DataBuffer::DataBuffer(double minVal, double maxVal,unsigned int trimSize):
    minVal_(minVal),maxVal_(maxVal),trimSize_(trimSize)
{
    //! @todo 1
    //! Start all the threads
    //! Syntax is
    //! thread_name = new std::thread(&ClassName::functionName,this)
    running_=true;
    removeValuesThread_ = new std::thread(&DataBuffer::removeValues,this);
    trimLengthThread_ = new std::thread(&DataBuffer::trimLength,this);
}

DataBuffer::~DataBuffer(){
    removeValuesThread_->join();
    trimLengthThread_->join();
    delete removeValuesThread_;
    delete trimLengthThread_;
}

void DataBuffer::removeValues(){
    while (running_) {
        // We use unique lock rather than locking and unlocking the mutex directly
        // http://www.cplusplus.com/reference/mutex/unique_lock/
        std::unique_lock<std::mutex> lck (mtx_);

        auto it = val_.begin();

        while ( it != val_.end()) {
            if (*it < minVal_ || *it > maxVal_) {
                val_.erase(it);
            } else {
                it++;
            }
        }

        lck.unlock();
        std::this_thread::sleep_for (std::chrono::milliseconds(20));
    }
}

void DataBuffer::addValues(vector<double> values){
  std::unique_lock<std::mutex> lck (mtx_);
  val_.insert(val_.end(), values.begin(),values.end());
  bubbleSort(val_);
}

void DataBuffer::trimLength(){

    while (running_) {
        std::unique_lock<std::mutex> lck (mtx_);
        while (val_.size()>trimSize_) {
            val_.erase(val_.begin());
        }
        lck.unlock();

        // This short delay prevents this thread from hard-looping and consuming too much cpu time
        // Using a condition_variable to make the thread wait provides a better solution to this problem
        std::this_thread::sleep_for (std::chrono::milliseconds(20));
    }
  
}

vector<double> DataBuffer::getValues(void){
    std::unique_lock<std::mutex> lck (mtx_);
    return val_;
}


void DataBuffer::bubbleSort(vector<double>& vec){

    unsigned long n = vec.size();
    bool swapped = false;
    while (!swapped) {
        swapped = true;
        for (unsigned int i = 1; i<n;i++){
            if (vec.at(i-1) > vec.at(i)){
                std::swap(vec.at(i-1), vec.at(i));
                swapped = false;
            }
        }
    }

}