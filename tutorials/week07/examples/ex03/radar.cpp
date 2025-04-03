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

  running_ = false; // We indicate the sensor should be running
  //We create the thread and push it to our vector of threads
  threads_.push_back(std::thread(&Radar::generateData,this));

}

Radar::~Radar(){
  running_=false;
  //Join threads
  for(auto & t: threads_){
    t.join();
  }
}

// THE START FUNCTION AND GENERATEDATA FUNCTION CAN BE 1 FUNCTION IF UPDATING running_ IS NOT NEEDED
// TUTORIAL.md SPECIFIED THAT THE THREAD OF EXECUTION IS INSIDE THE RADAR VIA THE start(radar.h) MEM FUNC???
void Radar::start(){
    
  // running_ = true;

  std::unique_lock<std::mutex> lck(mtxStart_);
  
  // running_ SHOULD BE HERE AND SHOULDN'T BE ATOMIC
  running_ = true;

  mtxStart_.unlock();

  cvStart_.notify_all();
}

void Radar::generateData(){

  std::unique_lock<std::mutex> lck(mtxStart_);
  //while (!running_) cvStart_.wait(lck);
  // OR
  cvStart_.wait(lck, [&](){return running_==true;});

  std::cout << "Start generating data" << std::endl;

  //generate random number of targets for each target (N) create Target containing random range and bearing between ^stored values
  // Info on dereferecing pointer https://stackoverflow.com/questions/27081035/why-should-i-dereference-a-pointer-before-calling-the-operator-in-c/27081074#27081074
  //! The check on the boolean enables us to exit this while loop when we need to terminate (and enable joining the thread)
  //! It is good practise to have this failsafe, so the thraeds can join rather than a while(true)
  while(running_){
    //We can now create a wait which will behave like a real sensor if we lock the mutex
    std::unique_lock<std::mutex> lck(mtx_);
    for (unsigned int i=0; i < numTarget_; i++){
      data_.at(i)=value_->operator()(*generator_); // GET NEW DATA AND STORE IT IN MEMVAR
    }
    ready_=true;
    lck.unlock();
    cv_.notify_all();
    //let's check here if we need to terminate before the sleep
    if(!running_){
     break;
    }
    std::this_thread::sleep_for (std::chrono::milliseconds(static_cast<int>(scanningTime_))); // SLEEP FOR THE FAKE SCANNING TIME 
  }
}

// BLOCKING CALL FUNCTION -> MAIN BLOCKED TILL DATA IS AVAILABLE
std::vector<double> Radar::getData(){
  //! We wait for the convar to release us, unless data is already ready
  std::unique_lock<std::mutex> lck(mtx_);
  //while (!ready_) cv_.wait(lck);
  // OR
  cv_.wait(lck, [&](){return ready_==true;});

  std::vector<double> data = data_; // IF DATA READY (NEW DATA RECEIVED), GET DATA FROM MEMVAR
  ready_=false;
  lck.unlock();
  //The below piece of code emulates a real sensor blocking call
  return data;
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
