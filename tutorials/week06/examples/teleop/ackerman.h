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

  //std::vector<std::thread> threads_; // We add threads onto a vector here to be able to terminate then in destructor
  std::thread* thread_;
  std::atomic<bool> running_;       // We use this to indicate the thread should be runnings
  std::mutex mtx_;

  pfms::PlatformType type_;
  Pipes* pipesPtr_;
  pfms::nav_msgs::Odometry odo_;
  double distanceTravelled_;
  double brake_;
  double steer_;
  double throttle_;

};

#endif // ACKERMAN_H
