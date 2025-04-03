#include <iostream>
#include <stdlib.h>
#include "radar.h"
#include <vector>
#include <chrono>
#include <thread>


Radar::Radar():
  scanningTime_(100),ready_(false),running_(false){

  //Generate first random value
  std::random_device rd;
  generator_= new std::mt19937(rd());
  value_ = new std::uniform_real_distribution<double>(0.1,maxDistance_);

  data_.resize(numTarget_);
}

Radar::~Radar(){

}

void Radar::start(){
  running_ = true;
  
  auto startTime = std::chrono::steady_clock::now();

  auto generateThread = new std::thread(&generateData, this);

  std::unique_lock<std::mutex> lck(mtx_);
  cv_.wait(lck, [this]{return ready_.load();});
}

void Radar::generateData(){
  //generate random number of targets for each target (N) create Target containing random range and bearing between ^stored values
  // Info on dereferecing pointer https://stackoverflow.com/questions/27081035/why-should-i-dereference-a-pointer-before-calling-the-operator-in-c/27081074#27081074
  while (running_){
    running_ = false;
    
    std::unique_lock<std::mutex> lck(mtx_);

    for (unsigned int i=0; i < numTarget_; i++){
      data_.at(i)=value_->operator()(*generator_);
    }
    lck.unlock();

    ready_ = true;
    cv_.notify_one();
  }
  std::this_thread::sleep_for (std::chrono::milliseconds(100));
}


std::vector<double> Radar::getData(){
  generateData();
  return data_;
}

void Radar::setScanningTime(double scanningTime){
  scanningTime_ = scanningTime;
}

double Radar::getScanningTime(void){
  return scanningTime_;
}

double Radar::getMaxDistance(void){
  return maxDistance_;
}
