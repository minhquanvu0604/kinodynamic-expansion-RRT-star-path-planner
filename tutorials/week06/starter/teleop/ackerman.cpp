#include "ackerman.h"
#include <iostream>
#include <cmath>

using std::cout;
using std::endl;
using pfms::commands::UGV;

Ackerman::Ackerman() :
    ugvSeq_(0),brake_(8000),steer_(0),throttle_(0)
{
    //! @todo
    // Create a `pipesPtr_` and start a thread on the `drive` function in constructor of `Ackerman`. 
    pipesPtr_ = new Pipes();

    thread_ = std::thread(&Ackerman::drive,this);
}

Ackerman::~Ackerman()
{
    //! @todo
    //! Force thread to stop and join thread in destructor.
    thread_.join();

    delete thread_;
}

void Ackerman::drive(void){

    while(running_){    
        //! @todo
        //! lock the mutex, create a `UGV` message and send it via the `pipesPtr_`
        {
            std::unique_lock<std::mutex> lck (mtx_);
            UGV ugv {
                    ugvSeq_++,
                    brake_,
                    steer_,
                    throttle_
                };
        
        // std::cout << "seq: " << ugvSeq_ << std::endl;
        // std::cout << "brake: " << brake_ << std::endl;
        // std::cout << "steer: " << steer_ << std::endl;
        // std::cout << "throttle: " << throttle_ << std::endl;


        pipesPtr_->send(ugv);
        }

        std::this_thread::sleep_for (std::chrono::milliseconds(50));
    }

}

void Ackerman::setCommand(double brake,double steer,double throttle){
   //! @todo
   //! store the values passed into member variables, don't forget the mutex.
   {
        std::unique_lock<std::mutex> lck (mtx_);

        brake_ = brake;
        steer_ = steer;
        throttle_ = throttle;
   }
}
