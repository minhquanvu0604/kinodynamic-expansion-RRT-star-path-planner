#ifndef ACKERMAN_H
#define ACKERMAN_H

#include <thread>
#include <mutex>
#include <atomic>
#include <vector>
#include "pfms_types.h"
#include "pipes.h"

class Ackerman
{
public:
  //Default constructor should set all sensor attributes to a default value
  Ackerman();
  ~Ackerman();

  void drive(void);
  void setCommand(double brake,double steer,double throttle);

private:
  unsigned int ugvSeq_;

  std::thread thread_; // We add threads onto a vector here to be able to terminate then in destructor
  std::atomic<bool> running_;       // We use this to indicate the thread should be runnings
  std::mutex mtx_;                  // Mutex used to lock internal member variables

  pfms::PlatformType type_;
  Pipes* pipesPtr_;
  pfms::nav_msgs::Odometry odo_;
  double distanceTravelled_;
  double brake_;
  double steer_;
  double throttle_;

};

#endif // ACKERMAN_H
