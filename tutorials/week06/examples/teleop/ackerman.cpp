#include "ackerman.h"
#include <iostream>
#include <cmath>

using std::cout;
using std::endl;
using pfms::commands::UGV;

Ackerman::Ackerman() :
    ugvSeq_(0),brake_(8000),steer_(0),throttle_(0)
{
    type_=pfms::PlatformType::ACKERMAN;
    pipesPtr_ = new Pipes();
    running_ = true;
    //thread_.push_back(std::thread(&Ackerman::drive,this));
    thread_ = new std::thread(&Ackerman::drive,this);
}

Ackerman::~Ackerman()
{
  running_=false;
//  std::this_thread::sleep_for (std::chrono::milliseconds(100));
//   //Join threads
//   for(auto & t: threads_){
//     t.join();
//   }
   thread_->join();
   delete thread_;
}

void Ackerman::drive(void){
    
    while(running_){    
        std::unique_lock<std::mutex> lck(mtx_);
        UGV ugv {ugvSeq_++,brake_,steer_,throttle_};
        mtx_.unlock();

        pipesPtr_->send(ugv);
        std::this_thread::sleep_for (std::chrono::milliseconds(50));
    }

}

void Ackerman::setCommand(double brake,double steer,double throttle){

    std::unique_lock<std::mutex> lck(mtx_);
    brake_=brake;
    steer_=steer;
    throttle_=throttle;
}
